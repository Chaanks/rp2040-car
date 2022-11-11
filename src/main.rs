//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::{
    entry,
    hal::{usb, Timer},
};
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::PwmPin;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// USB Communications Class Device support
use usbd_serial::SerialPort;

// Used to demonstrate writing formatted strings
use core::fmt::Write;
use heapless::String;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
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

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Init PWMs
    let mut pwm_slices = bsp::hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure PWM1 for motor 1
    let pwm = &mut pwm_slices.pwm1;
    pwm.set_ph_correct();
    pwm.enable();

    // Output channel B on PWM1 to GPIO 3
    let m1f_pwm = &mut pwm.channel_b;
    m1f_pwm.output_to(pins.gpio3);

    let m1b_pwm = &mut pwm.channel_a;
    m1b_pwm.output_to(pins.gpio2);

    // // Configure PWM1 for motor 1
    // let pwm = &mut pwm_slices.pwm1;
    // pwm.set_ph_correct();
    // pwm.enable();

    // M1 FP 3
    // M1 BP 2
    // M2 FP 6
    // M2 BP 7
    let mut led_pin = pins.led.into_push_pull_output();
    // let mut m1_f = pins.gpio3.into_push_pull_output();
    // let mut m1_b = pins.gpio2.into_push_pull_output();
    // let mut m2_f = pins.gpio6.into_push_pull_output();
    // let mut m2_b = pins.gpio7.into_push_pull_output();

    loop {
        info!("on!");
        led_pin.set_high().unwrap();
        for i in (0..65535).skip(1000) {
            m1f_pwm.set_duty(i);
            m1b_pwm.set_duty(0);
            delay.delay_us(200);
        }

        info!("off!");
        led_pin.set_low().unwrap();
        m1f_pwm.set_duty(0);
        m1b_pwm.set_duty(0);
        delay.delay_ms(2000);
    }
}
