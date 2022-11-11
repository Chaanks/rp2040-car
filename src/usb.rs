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

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut said_hello = false;

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Init PWMs
    let mut pwm_slices = bsp::hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure PWM4
    let pwm = &mut pwm_slices.pwm1;
    pwm.set_ph_correct();
    pwm.enable();

    // Output channel B on PWM4 to GPIO 25
    let channel_a = &mut pwm.channel_a;
    channel_a.output_to(pins.gpio2);

    let channel_b = &mut pwm.channel_b;
    channel_b.output_to(pins.gpio3);

    /*
    ENA: PIN2
    IN1: PIN4
    IN2: PIN6
    IN3: PIN7
    IN4: PIN8
    ENB: PIN3
    */

    let mut led_pin = pins.led.into_push_pull_output();
    let mut in_1 = pins.gpio4.into_push_pull_output();
    let mut in_2 = pins.gpio6.into_push_pull_output();
    let mut in_3 = pins.gpio7.into_push_pull_output();
    let mut in_4 = pins.gpio8.into_push_pull_output();

    led_pin.set_high().unwrap();
    let _ = serial.write(b"Hello, World!\r\n");

    loop {
        // info!("on!");
        // led_pin.set_high().unwrap();
        // in_1.set_low().unwrap();
        // in_2.set_high().unwrap();
        // for i in (0..65535).skip(1000) {
        //     channel_a.set_duty(i);
        //     delay.delay_us(200);
        // }

        // led_pin.set_high().unwrap();
        // in_3.set_low().unwrap();
        // in_4.set_high().unwrap();
        // channel_b.set_duty(60000);
        // delay.delay_ms(2000);

        // channel_b.set_duty(0);
        // delay.delay_ms(1000);

        // in_3.set_high().unwrap();
        // in_4.set_low().unwrap();
        // channel_b.set_duty(60000);
        // delay.delay_ms(2000);

        // info!("off!");
        // led_pin.set_low().unwrap();
        // in_1.set_low().unwrap();
        // in_2.set_low().unwrap();
        // in_3.set_high().unwrap();
        // in_4.set_low().unwrap();
        // delay.delay_ms(5000);

        // A welcome message at the beginning
        // if !said_hello && timer.get_counter() >= 2_000_000 {
        //     info!("in hello");
        //     said_hello = true;
        //     let _ = serial.write(b"Hello, World!\r\n");

        //     let time = timer.get_counter();
        //     let mut text: String<64> = String::new();
        //     writeln!(&mut text, "Current timer ticks: {}", time).unwrap();

        //     // This only works reliably because the number of bytes written to
        //     // the serial port is smaller than the buffers available to the USB
        //     // peripheral. In general, the return value should be handled, so that
        //     // bytes not transferred yet don't get lost.
        //     let _ = serial.write(text.as_bytes());
        // }

        // Check for new data
        // if usb_dev.poll(&mut [&mut serial]) {
        //     info!("receive data");
        //     let mut buf = [0u8; 64];
        //     match serial.read(&mut buf) {
        //         Err(_e) => {
        //             // Do nothing
        //         }
        //         Ok(0) => {
        //             // Do nothing
        //         }
        //         Ok(count) => {
        //             // Convert to upper case
        //             buf.iter_mut().take(count).for_each(|b| {
        //                 b.make_ascii_uppercase();
        //             });

        //             // Send back to the host
        //             let mut wr_ptr = &buf[..count];
        //             while !wr_ptr.is_empty() {
        //                 match serial.write(wr_ptr) {
        //                     Ok(len) => wr_ptr = &wr_ptr[len..],
        //                     // On error, just drop unwritten data.
        //                     // One possible error is Err(WouldBlock), meaning the USB
        //                     // write buffer is full.
        //                     Err(_) => break,
        //                 };
        //             }
        //         }
        //     }
        // }
    }
}

// End of file
