#![no_std]
#![no_main]

mod mpu6050;

use arduino_hal::{default_serial, prelude::*, I2c};
use mpu6050::{AccConfig, Dlpf, GyrConfig, MPU6050};
use panic_halt as _;
use ufmt::uwriteln;

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

    let mut serial = default_serial!(dp, pins, 57600);

    let mut i2c = I2c::new(
        dp.TWI,
        pins.a4.into_pull_up_input(),
        pins.a5.into_pull_up_input(),
        100000,
    );

    let mut mpu6050 = match MPU6050::new(&mut i2c, GyrConfig::Gyr250, AccConfig::Acc2g, Dlpf::Six) {
        Ok(v) => v,
        Err(e) => {
            uwriteln!(serial, "Error while creating MPU6050 object: {:?}", e).unwrap_infallible();
            panic!()
        }
    };

    match mpu6050.calibrate(&mut i2c) {
        Ok(_) => {}
        Err(e) => {
            uwriteln!(serial, "Error while Calibrating: {:?}", e).unwrap_infallible();
        }
    }

    loop {
        match mpu6050.read_data(&mut i2c) {
            Ok(_) => {}
            Err(e) => {
                uwriteln!(serial, "Error reading Sensor data: {:?}", e).unwrap_infallible();
                arduino_hal::delay_ms(100);
                continue;
            }
        }
        mpu6050.print(&mut serial);
        led.toggle();
        arduino_hal::delay_ms(250);
    }
}
