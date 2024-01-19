#![no_std]
#![no_main]

use core::{panic::PanicInfo, sync::atomic::{AtomicBool, AtomicU8}};

use cyw43_pio::PioSpi;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_net::{StaticConfigV4, Ipv4Address, Ipv4Cidr, Stack, StackResources, tcp::TcpSocket};
use embassy_rp::{pio::{Pio, InterruptHandler, Instance, StateMachine, Common, PioPin, Config, FifoJoin, ShiftConfig, ShiftDirection, Direction}, bind_interrupts, peripherals::{PIO0, PIN_23, PIN_25, DMA_CH0, PIN_29, PIN_24}, PeripheralRef, dma::{AnyChannel, Channel}, into_ref, Peripheral, gpio::{Output, Level, AnyPin}};
use embassy_sync::{mutex::Mutex, blocking_mutex::raw::ThreadModeRawMutex};
use embassy_time::{Timer, Duration};
use embedded_io_async::Read;
use fixed::types::U24F8;
use heapless::Vec;
use libm::{expf, floorf};
use portable_atomic::Ordering;
use static_cell::StaticCell;

mod credentials;

use credentials::*;

const LED_COUNT: usize = 60;

static LEDS_ENABLED: AtomicBool = AtomicBool::new(true);
static COLORS: Mutex<ThreadModeRawMutex, [[u8; 3]; 2]> = Mutex::new([
    [255, 0, 0],
    [0, 0, 255],
]);

#[panic_handler]
fn panic(_: &PanicInfo) -> ! {
    cortex_m::asm::udf();
}

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

struct Ws2812<'d, P: Instance, const S: usize, const N: usize> {
    dma: PeripheralRef<'d, AnyChannel>,
    sm: StateMachine<'d, P, S>,
}

impl<'d, P: Instance, const S: usize, const N: usize> Ws2812<'d, P, S, N> {
    pub fn new(pio: &mut Common<'d, P>, mut sm: StateMachine<'d, P, S>, dma: impl Peripheral<P = impl Channel> + 'd, pin: impl PioPin) -> Self {
        into_ref!(dma);

        let ws2812_program = pio_proc::pio_asm!(
            ".side_set 1",
            ".wrap_target",
            "bitloop:",
            "    out x, 1        side 0 [4]",
            "    jmp !x do_zero  side 1 [1]",
            "    jmp bitloop     side 1 [2]",
            "do_zero:",
            "    nop             side 0 [2]",
            ".wrap",
        )
        .program;

        // let ws2812_program = pio_proc::pio_asm!(
        //     ".side_set 1",
        //     ".wrap_target",
        //     "    nop side 1",
        //     "    nop side 0",
        //     ".wrap",
        // ).program;

        let mut cfg = Config::default();

        let out_pin = pio.make_pio_pin(pin);
        cfg.set_out_pins(&[&out_pin]);
        cfg.set_set_pins(&[&out_pin]);

        cfg.use_program(&pio.load_program(&ws2812_program), &[&out_pin]);

        // 125 MHz / 8 MHz
        cfg.clock_divider = U24F8::from_num(125) / U24F8::from_num(8);

        cfg.fifo_join = FifoJoin::TxOnly;
        cfg.shift_out = ShiftConfig {
            auto_fill: true,
            threshold: 24,
            direction: ShiftDirection::Left,
        };

        sm.set_config(&cfg);
        sm.set_pin_dirs(Direction::Out, &[&out_pin]);
        sm.set_enable(true);

        Self {
            dma: dma.map_into(),
            sm,
        }
    }

    pub async fn write(&mut self, colors: &[u32; N]) {
        self.sm.tx().dma_push(self.dma.reborrow(), colors).await;
    }
}

#[embassy_executor::task]
async fn wifi_task(runner: cyw43::Runner<'static, Output<'static, PIN_23>, PioSpi<'static, PIN_25, PIO0, 0, DMA_CH0>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<cyw43::NetDriver<'static>>) -> ! {
    stack.run().await
}

#[embassy_executor::task]
async fn wifi_setup(spawner: Spawner, power: PIN_23, debug: AnyPin, spi: PioSpi<'static, PIN_25, PIO0, 0, DMA_CH0>) {
    let mut debug = Output::new(debug, Level::High);
    let fw = include_bytes!("../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../cyw43-firmware/43439A0_clm.bin");

    let cyw_pwr = Output::new(power, Level::Low);
    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, cyw_pwr, spi, fw).await;

    spawner.spawn(wifi_task(runner)).unwrap();

    control.init(clm).await;
    control.set_power_management(cyw43::PowerManagementMode::PowerSave).await;

    //let config = embassy_net::Config::dhcpv4(Default::default());
    let config = embassy_net::Config::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(Ipv4Address::new(192, 168, 0, 176), 24),
        dns_servers: Vec::from_slice(&[Ipv4Address::new(1, 1, 1, 1), Ipv4Address::new(1, 0, 0, 1)]).unwrap(),
        gateway: Some(Ipv4Address::new(192, 168, 0, 1)),
    });

    let seed = 0xdf59_9ecd_4999_9e36;

    static STACK: StaticCell<Stack<cyw43::NetDriver<'static>>> = StaticCell::new();
    static RESOURCES: StaticCell<StackResources<2>> = StaticCell::new();
    let stack = &*STACK.init(Stack::new(
        net_device,
        config,
        RESOURCES.init(StackResources::new()),
        seed,
    ));

    spawner.spawn(net_task(stack)).unwrap();

    loop {
        match control.join_wpa2(WIFI_SSID, WIFI_PASSWORD).await {
            Ok(_) => break,
            Err(_) => (),
        }
    }

    let mut tcp_rx = [0; 4096];
    let mut tcp_tx = [0; 4096];
    let mut buf = [0; 4096];

    'tcploop: loop {

        let mut socket = TcpSocket::new(stack, &mut tcp_rx, &mut tcp_tx);
        socket.set_timeout(Some(Duration::from_secs(10)));

        control.gpio_set(0, true).await;

        if let Err(_) = socket.accept(1234).await {
            continue;
        }

        control.gpio_set(0, false).await;

        'msgloop: while let Ok(_) = {
            debug.set_high();
            socket.read_exact(&mut buf[..1]).await
        } {
            debug.set_low();
            match buf[0] {
                v @ (0 | 1) => LEDS_ENABLED.store(v != 0, Ordering::Relaxed),
                2 => {
                    if let Err(_) = socket.read_exact(&mut buf[..4]).await {
                        socket.abort();
                        continue 'tcploop;
                    }
                    let mut colors = COLORS.lock().await;
                    if buf[0] as usize >= colors.len() {
                        continue 'msgloop;
                    }
                    colors[buf[0] as usize] = buf[1..4].try_into().unwrap();
                },
                _ => (),
            }
        }
        debug.set_low();
        socket.abort();
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p: embassy_rp::Peripherals = embassy_rp::init(Default::default());

    let Pio { mut common, sm0, sm1, irq0, .. } = Pio::new(p.PIO0, Irqs);

    let cyw_cs = Output::new(p.PIN_25, Level::High);
    let cyw_spi = PioSpi::new(&mut common, sm0, irq0, cyw_cs, p.PIN_24, p.PIN_29, p.DMA_CH0);
    spawner.spawn(wifi_setup(spawner, p.PIN_23, p.PIN_14.into(), cyw_spi)).unwrap();

    let mut ws2812 = Ws2812::new(&mut common, sm1, p.DMA_CH1, p.PIN_15);

    let mut values = [0f32; LED_COUNT];
    let mut buf_a = [0; LED_COUNT];
    let mut buf_b = [0; LED_COUNT];

    let mut tx_buf = &mut buf_a;
    let mut process_buf = &mut buf_b;

    const SPEED: f32 = 0.005;
    let damping = expf(-SPEED / 8.);
    let mut posf = 0.;

    let mut colors_cached = *COLORS.lock().await;

    loop {
        join(async {
            ws2812.write(&tx_buf).await;
            Timer::after_micros(550).await;
        }, async {
            posf += SPEED;
            posf %= LED_COUNT as f32;
            let pos = floorf(posf) as usize;

            for i in 0..LED_COUNT {
                values[i] *= damping;
            }

            let f = posf % 1.;
            values[pos] = values[pos].max(1. - f);
            values[(pos + 1) % LED_COUNT] = values[(pos + 1) % LED_COUNT].max(f);

            if let Ok(colors) = COLORS.try_lock() {
                colors_cached = *colors;
            }
            let [a, b] = colors_cached;

            #[allow(dead_code)]
            const RED: [u8; 3] = [255, 0, 0];
            #[allow(dead_code)]
            const GREEN: [u8; 3] = [0, 255, 0];
            #[allow(dead_code)]
            const BLUE: [u8; 3] = [0, 0, 255];
            #[allow(dead_code)]
            const MAGENTA: [u8; 3] = [255, 0, 255];
            #[allow(dead_code)]
            const YELLOW: [u8; 3] = [255, 255, 0];
            #[allow(dead_code)]
            const CYAN: [u8; 3] = [0, 255, 255];

            if LEDS_ENABLED.load(Ordering::Relaxed) {
                for i in 0..LED_COUNT {
                    process_buf[i] = rgb(
                        (a[0] as u16 * f2i(values[i] * 255.) as u16 / 255).max(b[0] as u16 * f2i(values[(i + LED_COUNT / 2) % LED_COUNT] * 255.) as u16 / 255) as u8,
                        (a[1] as u16 * f2i(values[i] * 255.) as u16 / 255).max(b[1] as u16 * f2i(values[(i + LED_COUNT / 2) % LED_COUNT] * 255.) as u16 / 255) as u8,
                        (a[2] as u16 * f2i(values[i] * 255.) as u16 / 255).max(b[2] as u16 * f2i(values[(i + LED_COUNT / 2) % LED_COUNT] * 255.) as u16 / 255) as u8,
                    );
                }
            } else {
                for i in 0..LED_COUNT {
                    process_buf[i] = 0;
                }
            }
        }).await;

        core::mem::swap(&mut tx_buf, &mut process_buf);
    }
}

const fn rgb(r: u8, g: u8, b: u8) -> u32 {
    (r as u32) << 16 | (g as u32) << 24 | (b as u32) << 8
}

fn f2i(x: f32) -> u8 {
    floorf(x) as u8
}
