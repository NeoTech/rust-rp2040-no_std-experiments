#![allow(dead_code)]
#![allow(unused_imports)]
#![no_std]
#![no_main]


use core::fmt::Write as core_write;
use core::str;
use cortex_m::asm;
use panic_halt as _;
use rp2040_hal as hal;
use hal::pac;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::InputPin;
use embedded_hal::serial::Read;
use embedded_hal::serial::Write;
use rp2040_hal::clocks::Clock;

use hal::usb::UsbBus;
use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::SerialPort;



#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

#[rp2040_hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let sio = hal::Sio::new(pac.SIO);
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();
    let mut _delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let mut _led_pin = pins.gpio25.into_push_pull_output();
    
    // Create a USB bus allocator
    let usb_clock = clocks.usb_clock;
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        usb_clock,
        true,
        &mut pac.RESETS));

    let mut serial = SerialPort::new(&usb_bus);
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST1234")
        .device_class(2)
        .build();

  
    let mut buffer = [0u8; 128];
    let mut buf_index = 0;
    loop {
        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }
        let mut buf = [0; 128];
        match serial.read(&mut buf) {
            Ok(count) => {
                let received_data = &buf[..count];
                serial.write(received_data).ok();
                if received_data[0] == 8 {
                    if buf_index > 0 {
                        buf_index -= 1;
                    }
                } else {
                    for byte in received_data {
                        if buf_index < buffer.len() {
                            buffer[buf_index] = *byte;
                            buf_index += 1;
                        }
                    }
                }
                if buf_index > 0 && buffer[buf_index - 1] == b'\n' {
                    // Write buffer to serial port
                    let received_data = &buffer[..buf_index];
                    // Trim string from linefeed characters
                    let received_data = str::from_utf8(received_data).unwrap().trim();
                    // convert string to u8
                    match received_data {
                        "LED_ON" => {
                            _led_pin.set_high().ok();
                            _delay.delay_ms(100);
                            _led_pin.set_low().ok();
                            _delay.delay_ms(100);
                            _led_pin.set_high().ok();
                            _delay.delay_ms(100);
                            _led_pin.set_low().ok();
                            _delay.delay_ms(100);
                            _led_pin.set_high().ok();
                        },
                        "LED_OFF" => {
                            _led_pin.set_low().ok();
                            _delay.delay_ms(100);
                            _led_pin.set_high().ok();
                            _delay.delay_ms(100);
                            _led_pin.set_low().ok();
                            _delay.delay_ms(100);
                            _led_pin.set_high().ok();
                            _delay.delay_ms(100);
                            _led_pin.set_low().ok();
                        },
                        "HELP" => {
                            serial.write(b"LED_ON => Turns LED on\r\n").ok();
                            serial.write(b"LED_OFF => Turns LED off\r\n").ok();
                            serial.write(b"HELP => Shows this\r\n").ok();
                        },
                        _ => {
                            let received_data = received_data.as_bytes();
                            serial.write(b"DEBUG:\r\n").ok();
                            serial.write(received_data).ok();
                            serial.write(b"\r\n").ok();
                            serial.write(b"------\r\n").ok();
                            serial.write(b"UNKNOWN COMMAND\r\n").ok();
                        }
                    }
                    buf_index = 0;
                }
            },
            Err(_) => {}
        }
    }
}

#[no_mangle]
#[link_section = ".rodata.str1.1"]
pub static _DEVICE_NAME: [u8; 6] = *b"usbser";
#[no_mangle]
#[link_section = ".rodata.str1.1"]
pub static _MANUFACTURER: [u8; 12] = *b"Roughedge AB";
#[no_mangle]
#[link_section = ".rodata.str1.1"]
pub static _PRODUCT_NAME: [u8; 25] = *b"SerialPort File interface";