#![feature(abi_avr_interrupt)]
#![no_std]
#![no_main]

mod millis;
mod mpu6050;

use arduino_hal::{default_serial, delay_ms, I2c};
use mpu6050::{AccConfig, Dlpf, GyrConfig, Measurements, MPU6050};
use panic_halt as _;

use crate::millis::{millis, millis_init};

#[arduino_hal::entry]
fn main() -> ! {
    let dp = match arduino_hal::Peripherals::take() {
        Some(periph) => periph,
        None => {
            panic!();
        }
    };
    let pins = arduino_hal::pins!(dp);
    let mut led = pins.d13.into_output();

    let mut serial = default_serial!(dp, pins, 115200);

    millis_init(dp.TC0);
    // Enable interrupts globally
    unsafe { avr_device::interrupt::enable() };

    let mut i2c = I2c::new(
        dp.TWI,
        pins.a4.into_pull_up_input(),
        pins.a5.into_pull_up_input(),
        100000,
    );

    let mut mpu6050 = match MPU6050::new(&mut i2c, GyrConfig::Gyr250, AccConfig::Acc2g, Dlpf::Two) {
        Ok(v) => {
            led.set_low();
            v
        }
        Err(_) => loop {
            led.set_high();
            delay_ms(1000);
            led.set_low();
            delay_ms(250);
        },
    };

    match mpu6050.calibrate(&mut i2c) {
        Ok(_) => {
            led.set_low();
        }
        Err(_) => loop {
            led.set_high();
            delay_ms(250);
            led.set_low();
            delay_ms(250);
        },
    }

    let mut prev_time = millis();

    loop {
        match mpu6050.read_data(&mut i2c) {
            Ok(_) => {}
            Err(_) => {
                led.set_high();
                delay_ms(50);
                led.set_low();
                delay_ms(50);
                continue;
            }
        }

        let now = millis();
        if now - prev_time >= 10 {
            serial.write_byte(0xAA);
            serial.write_byte(0xBB);
            serial.write_byte(0xCC);
            serial.write_byte(0xDD);

            let mut bytes = mpu6050.get_data(Measurements::CompRoll).to_be_bytes();
            for byte in bytes.iter() {
                serial.write_byte(*byte);
            }

            bytes = mpu6050.get_data(Measurements::CompPitch).to_be_bytes();
            for byte in bytes.iter() {
                serial.write_byte(*byte);
            }

            prev_time = now;
        }
    }
}
