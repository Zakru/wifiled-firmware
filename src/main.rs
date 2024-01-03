#![no_std]
#![no_main]

use core::panic::PanicInfo;

use embassy_executor::Spawner;
use embassy_futures::{join::join, yield_now};
use embassy_rp::{pio::{Pio, InterruptHandler, Instance, StateMachine, Common, PioPin, Config, FifoJoin, ShiftConfig, ShiftDirection, Direction}, bind_interrupts, peripherals::PIO0, PeripheralRef, dma::{AnyChannel, Channel}, into_ref, Peripheral, gpio::{Output, Level}};
use embassy_time::Timer;
use fixed::types::U24F8;
use libm::{expf, floorf};

const LED_COUNT: usize = 60;

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

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p: embassy_rp::Peripherals = embassy_rp::init(Default::default());

    let Pio { mut common, sm0, .. } = Pio::new(p.PIO0, Irqs);
    let mut led = Output::new(p.PIN_25, Level::Low);

    let mut ws2812 = Ws2812::new(&mut common, sm0, p.DMA_CH0, p.PIN_15);

    let mut values = [0f32; LED_COUNT];
    let mut buf_a = [0; LED_COUNT];
    let mut buf_b = [0; LED_COUNT];

    let mut tx_buf = &mut buf_a;
    let mut process_buf = &mut buf_b;

    const SPEED: f32 = 0.005;
    let damping = expf(-SPEED / 8.);
    let mut posf = 0.;

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

            #[allow(dead_code)]
            const RED: [u8; 3] = [1, 0, 0];
            #[allow(dead_code)]
            const GREEN: [u8; 3] = [0, 1, 0];
            #[allow(dead_code)]
            const BLUE: [u8; 3] = [0, 0, 1];
            #[allow(dead_code)]
            const MAGENTA: [u8; 3] = [1, 0, 1];
            #[allow(dead_code)]
            const YELLOW: [u8; 3] = [1, 1, 0];
            #[allow(dead_code)]
            const CYAN: [u8; 3] = [0, 1, 1];

            const COL1: [u8; 3] = RED;
            const COL2: [u8; 3] = BLUE;

            for i in 0..LED_COUNT {
                process_buf[i] = rgb(
                    (COL1[0] * f2i(values[i] * 255.)).max(COL2[0] * f2i(values[(i + LED_COUNT / 2) % LED_COUNT] * 255.)),
                    (COL1[1] * f2i(values[i] * 255.)).max(COL2[1] * f2i(values[(i + LED_COUNT / 2) % LED_COUNT] * 255.)),
                    (COL1[2] * f2i(values[i] * 255.)).max(COL2[2] * f2i(values[(i + LED_COUNT / 2) % LED_COUNT] * 255.)),
                );
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
