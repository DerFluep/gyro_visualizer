#![no_std]
#![no_main]

mod mpu6050;

use arduino_hal::{default_serial, prelude::*, I2c};
use panic_halt as _;

use mpu6050::{AccConfig, GyrConfig, MPU6050};

use crate::mpu6050::Dlpf;

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    let mut i2c = I2c::new(
        dp.TWI,
        pins.a4.into_pull_up_input(),
        pins.a5.into_pull_up_input(),
        50000,
    );
    let mut serial = default_serial!(dp, pins, 57600);

    let mut mpu6050 = match MPU6050::new(&mut i2c, GyrConfig::Gyr250, AccConfig::Acc2g, Dlpf::Zero)
    {
        Ok(v) => v,
        Err(e) => {
            ufmt::uwriteln!(&mut serial, "Error while creating MPU6050 object: {:?}", e)
                .unwrap_infallible();
            panic!()
        }
    };

    match mpu6050.calibrate(&mut i2c) {
        Ok(_) => {}
        Err(e) => {
            ufmt::uwriteln!(serial, "Error while Calibrating: {:?}", e).unwrap_infallible();
        }
    }

    loop {
        match mpu6050.read_data(&mut i2c) {
            Ok(_) => {}
            Err(e) => {
                ufmt::uwriteln!(serial, "Error reading Sensor data: {:?}", e).unwrap_infallible();
                arduino_hal::delay_ms(100);
                continue;
            }
        }
        mpu6050.print(&mut serial);
        arduino_hal::delay_ms(250);
    }
}
