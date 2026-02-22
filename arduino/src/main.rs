#![no_std]
#![no_main]

mod mpu6050;

use arduino_hal::{default_serial, prelude::*, I2c};
use panic_halt as _;

use mpu6050::{AccConfig, GyrConfig, Sensor, MPU6050};
use ufmt_float::uFmt_f32;

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    let i2c = I2c::new(
        dp.TWI,
        pins.a4.into_pull_up_input(),
        pins.a5.into_pull_up_input(),
        50000,
    );
    let mut serial = default_serial!(dp, pins, 57600);

    let mut mpu6050 = MPU6050::new(i2c, GyrConfig::Gyr250, AccConfig::Acc2g);

    loop {
        mpu6050.read_data();
        let mut data = mpu6050.get_data(Sensor::GyrX);
        let mut data_sym = "";
        if data < 0.0 {
            data_sym = "-";
            data = data * -1.0;
        }
        ufmt::uwriteln!(&mut serial, "Acc X: {}{}", data_sym, uFmt_f32::Two(data))
            .unwrap_infallible();
        arduino_hal::delay_ms(100);
    }
}
