#![no_std]
#![no_main]

mod mpu6050;

use arduino_hal::{default_serial, prelude::*, I2c};
use panic_halt as _;

use mpu6050::{AccConfig, GyrConfig, MPU6050};

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

    let mut mpu6050 = MPU6050::new(i2c, GyrConfig::Gyr250, AccConfig::Acc2g);

    loop {
        let datas = mpu6050.read_data(MPU6050::ACC);
        for data in datas.iter() {
            ufmt::uwriteln!(&mut serial, "{}", data).unwrap_infallible();
        }
        arduino_hal::delay_ms(100);
    }
}
