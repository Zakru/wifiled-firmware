#![no_std]
#![no_main]

use core::panic::PanicInfo;

use cortex_m::singleton;
use cortex_m_rt::entry;

use libm::{expf, floorf};
use rp2040_hal as hal;

use hal::{
    clocks::{init_clocks_and_plls, Clock},
    dma::{single_buffer, DMAExt},
    gpio::{FunctionPio0, Pin},
    pac,
    pio::{PIOBuilder, PIOExt, PinDir, PinState},
    watchdog::Watchdog,
    Sio,
};
use usb_device::class_prelude::UsbBusAllocator;
use usbd_serial::SerialPort;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

const EXTERNAL_CRYSTAL_FREQUENCY_HZ: u32 = 12_000_000;

const LED_COUNT: usize = 60;

#[panic_handler]
fn panic(_: &PanicInfo) -> ! {
    cortex_m::asm::udf();
}

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let clocks = init_clocks_and_plls(
        EXTERNAL_CRYSTAL_FREQUENCY_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    /*let mut serial = SerialPort::new(&usb_bus);
    serial.*/

    let mut led_pin = pins.gpio25.into_push_pull_output();
    let led_pin_num = led_pin.id().num;
    let neo_pin: Pin<_, FunctionPio0, _> = pins.gpio15.into_function();
    let neo_pin_num = neo_pin.id().num;

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let neo_program = pio_proc::pio_asm!(
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
    // let neo_program = pio_proc::pio_asm!(
    //     ".side_set 1",
    //     ".wrap_target",
    //     "    nop side 1",
    //     "    nop side 0",
    //     ".wrap",
    // ).program;
    let installed = pio.install(&neo_program).unwrap();
    let (mut sm, _rx, mut tx) = PIOBuilder::from_program(installed)
        .out_shift_direction(hal::pio::ShiftDirection::Left)
        .autopull(true)
        .pull_threshold(24)
        .side_set_pin_base(neo_pin_num)
        .clock_divisor_fixed_point(15, 160)
        .build(sm0);
    sm.set_pindirs([(neo_pin_num, PinDir::Output)]);
    sm.set_pins([(neo_pin_num, PinState::Low)]);
    sm.start();

    // loop {
    //     tx.write(0x00ff0000);
    //     tx.write(0xff000000);
    //     tx.write(0x0000ff00);
    //     led_pin.set_high().unwrap();
    //     delay.delay_ms(500);
    //     led_pin.set_low().unwrap();
    //     delay.delay_ms(500);
    // }

    let dma = pac.DMA.split(&mut pac.RESETS);

    let mut values = [0f32; LED_COUNT];
    let mut tx_buf = singleton!(: [u32; LED_COUNT] = [0; LED_COUNT]).unwrap();
    let mut dma_ch = dma.ch0;

    const SPEED: f32 = 0.005;
    let damping = expf(-SPEED / 8.);
    let mut posf = 0.;

    loop {
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
            tx_buf[i] = rgb(
                (COL1[0] * f2i(values[i] * 255.)).max(COL2[0] * f2i(values[(i + LED_COUNT / 2) % LED_COUNT] * 255.)),
                (COL1[1] * f2i(values[i] * 255.)).max(COL2[1] * f2i(values[(i + LED_COUNT / 2) % LED_COUNT] * 255.)),
                (COL1[2] * f2i(values[i] * 255.)).max(COL2[2] * f2i(values[(i + LED_COUNT / 2) % LED_COUNT] * 255.)),
            );
        }

        let tx_transfer = single_buffer::Config::new(dma_ch, tx_buf, tx).start();
        (dma_ch, tx_buf, tx) = tx_transfer.wait();
    }
}

const fn rgb(r: u8, g: u8, b: u8) -> u32 {
    (r as u32) << 16 | (g as u32) << 24 | (b as u32) << 8
}

fn f2i(x: f32) -> u8 {
    floorf(x) as u8
}
