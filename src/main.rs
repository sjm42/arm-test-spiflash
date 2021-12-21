// main.rs

#![no_std]
#![no_main]
#![deny(unsafe_code)]
// #![deny(warnings)]

// https://www.st.com/en/microcontrollers-microprocessors/stm32f411ce.html

use core::fmt::Write; // for pretty formatting of the serial output
use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use panic_halt as _;

use stm32f4xx_hal as hal;

use hal::pac::USART1;
use hal::spi::{Mode, Phase, Polarity};
use hal::{gpio::*, pac, prelude::*, serial, spi::Spi};

use spi_memory::prelude::*;
use spi_memory::series25::Flash;

const MY_ADDR: u32 = 0x02_08_00;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::peripheral::Peripherals::take().unwrap();

    // Enable clock outputs 1+2
    // With 100MHz sysclk we should see 8/5 = 1.6 MHz on MCO1 and 100/5 = 20MHz on MCO2
    dp.RCC.cfgr.modify(|_r, w| {
        w.mco1pre().div5();
        w.mco1().hsi();
        w.mco2pre().div5();
        w.mco2().sysclk();
        w
    });

    let rcc = dp.RCC.constrain();
    let pa = dp.GPIOA.split();
    let pc = dp.GPIOC.split();

    // Setup system clock to 72 MHz
    let sysclk = 72;
    let clocks = rcc.cfgr.sysclk(sysclk.mhz()).freeze();

    // Create a delay abstraction based on SysTick
    let mut delay = hal::delay::Delay::new(cp.SYST, &clocks);

    // Clock outputs are as alt functions on MCO1=PA8, MCO2=PC9
    let _mco1 = pa.pa8.into_alternate::<0>().set_speed(Speed::VeryHigh);
    let _mco2 = pc.pc9.into_alternate::<0>().set_speed(Speed::VeryHigh);

    // On Blackpill stm32f411 user led is on PC13, active low
    let mut led = pc.pc13.into_push_pull_output().erase();

    // User button on Blackpill is on PA0, active low
    let button = pa.pa0.into_pull_up_input();

    let ser_tx_pin = pa.pa9.into_alternate::<7>();
    let _ser_rx_pin = pa.pa10.into_alternate::<7>();

    // default is 115200 bps, 8N1
    let ser_cfg = serial::config::Config::default().wordlength_8();
    let mut ser_tx = serial::Serial::tx(dp.USART1, ser_tx_pin, ser_cfg, clocks).unwrap();

    macro_rules! serpr {
        ($ser:expr, $($arg : expr), *) =>
            {let _ = write!($ser, $($arg),*); }
    }

    serpr!(ser_tx, "\r\n\n*** Waiting for button...\r\n");

    while button.is_high() {
        delay.delay_ms(100u32);
    }

    serpr!(ser_tx, "\r\n\n*** Starting spiflash\r\n");
    serpr!(ser_tx, "\r\nRunning with {} MHz sysclk\r\n", sysclk);
    serpr!(ser_tx, "Configure cs...\r\n");
    // https://docs.zephyrproject.org/2.6.0/boards/arm/blackpill_f411ce/doc/index.html
    // SPI1 CS/SCK/MISO/MOSI : PA4/PA5/PA6/PA7 (Routed to footprint for external flash)
    let cs = {
        let mut cs = pa.pa4.into_push_pull_output().erase();
        cs.set_high(); // deselect
        cs
    };

    let mut buf = [0u8; 256];
    for (i, b) in buf.iter_mut().enumerate() {
        *b = i as u8;
    }
    serpr!(ser_tx, "Hex dump of write buf:\r\n");
    hex_dump(&mut ser_tx, &buf);

    serpr!(ser_tx, "Init spi...\r\n");
    let spi = {
        let sck = pa.pa5.into_alternate::<5>();
        let miso = pa.pa6.into_alternate::<5>();
        let mosi = pa.pa7.into_alternate::<5>();

        Spi::new(
            dp.SPI1,
            (sck, miso, mosi),
            Mode {
                polarity: Polarity::IdleHigh,
                phase: Phase::CaptureOnSecondTransition,
            },
            16.mhz(),
            clocks,
        )
    };

    serpr!(ser_tx, "Flash init...\r\n");
    let delay_us = |us| delay.delay_us(us);
    // let delay_us = |_us| ();
    let mut flash = Flash::init(spi, cs, delay_us).unwrap();

    serpr!(ser_tx, "Flash reset...\r\n");
    flash.reset_device().unwrap();

    serpr!(ser_tx, "Read JEDEC id...\r\n");
    let jedec_id = flash.read_jedec_id().unwrap();
    serpr!(ser_tx, "Flash jedec id: {:?}\r\n", jedec_id);

    #[cfg(feature = "write_flash")]
    {
        serpr!(ser_tx, "Using flash address: {:06x}\r\n", MY_ADDR);
        serpr!(ser_tx, "Flash erase...\r\n");
        let mut w = flash.erase_sectors(MY_ADDR, 1).unwrap();
        serpr!(ser_tx, "Wait cycles: {}\r\n", w);

        serpr!(ser_tx, "Flash write...\r\n");
        w = flash.write_bytes(MY_ADDR, &mut buf).unwrap();
        serpr!(ser_tx, "Wait cycles: {}\r\n", w);
    }

    buf.iter_mut().map(|b| *b = 0).count();
    serpr!(ser_tx, "Flash read dump:\r\n");
    flash.read(MY_ADDR, &mut buf).unwrap();
    hex_dump(&mut ser_tx, &buf);

    serpr!(ser_tx, "Flash read word by word:\r\n");
    let mut buf1 = [0u8; 4];
    let mut addr = 0u32;
    let mut n_row = 0usize;
    while addr < buf.len() as u32 {
        buf1.iter_mut().map(|b| *b = 0).count();
        flash.read(MY_ADDR + addr, &mut buf1).unwrap();
        serpr!(
            ser_tx,
            "{:02x}{:02x}{:02x}{:02x} ",
            buf1[0],
            buf1[1],
            buf1[2],
            buf1[3]
        );
        n_row += 4;
        if n_row >= ROW_SZ {
            serpr!(&mut ser_tx, "\r\n");
            n_row = 0;
        }
        addr += 4;
    }
    serpr!(&mut ser_tx, "\r\n");

    // st = flash.read_status().unwrap();
    // serpr!(ser_tx, "- status: {:?}\r\n", st);

    delay.delay_ms(1_000u32);
    let mut i = 0u8;
    loop {
        serpr!(ser_tx, "{} ", i);
        i += 1; // let it wrap
        for _i in 1..=3 {
            set_led(&mut led, true);
            delay.delay_ms(100_u32);

            set_led(&mut led, false);
            delay.delay_ms(400_u32);
        }
        delay.delay_ms(1000_u32);
    }
}

const ROW_SZ: usize = 32;

fn hex_dump(serial: &mut serial::Tx<USART1>, buf: &[u8]) {
    let mut offset: usize = 0;
    let len = buf.len();
    let mut stop = false;
    while !stop {
        let mut end = offset + ROW_SZ;
        if end > len {
            stop = true;
            end = len;
        }
        let slice = &buf[offset..end];
        offset += ROW_SZ;
        for c in slice {
            let _ = write!(serial, "{:02x} ", c);
        }
        let _ = write!(serial, "\r\n");
    }
}

fn set_led(led: &mut dyn OutputPin<Error = core::convert::Infallible>, state: bool) {
    let active_low = true;

    if state ^ active_low {
        let _ = led.set_high();
    } else {
        let _ = led.set_low();
    }
}
// EOF
