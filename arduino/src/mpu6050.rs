use arduino_hal::{i2c::Error, prelude::*, I2c};
use ufmt_float::uFmt_f32;

#[allow(dead_code)]
pub enum AccConfig {
    Acc2g,
    Acc4g,
    Acc8g,
    Acc16g,
}

#[allow(dead_code)]
pub enum GyrConfig {
    Gyr250,
    Gyr500,
    Gyr1000,
    Gyr2000,
}

#[allow(dead_code)]
pub enum Sensor {
    AccX,
    AccY,
    AccZ,
    Temp,
    GyrX,
    GyrY,
    GyrZ,
    AccXRaw,
}

pub struct MPU6050 {
    gyr_conf: GyrConfig,
    acc_conf: AccConfig,
    raw_data: [u8; 14],
    data: [i16; 7],
    offsets: [i16; 7],
}

impl MPU6050 {
    #[allow(dead_code)]
    pub const MPU_ADR: u8 = 0x68;
    #[allow(dead_code)]
    pub const SENSORS_START: u8 = 0x3B;
    #[allow(dead_code)]
    pub const ACC: u8 = 0x3B;
    #[allow(dead_code)]
    pub const ACCX: u8 = 0x3B;
    #[allow(dead_code)]
    pub const ACCY: u8 = 0x3D;
    #[allow(dead_code)]
    pub const ACCZ: u8 = 0x3F;
    #[allow(dead_code)]
    pub const TEMP: u8 = 0x41;
    #[allow(dead_code)]
    pub const GYR: u8 = 0x43;
    #[allow(dead_code)]
    pub const GYRX: u8 = 0x43;
    #[allow(dead_code)]
    pub const GYRY: u8 = 0x45;
    #[allow(dead_code)]
    pub const GYRZ: u8 = 0x47;

    pub fn new(i2c: &mut I2c, gyr_conf: GyrConfig, acc_conf: AccConfig) -> Result<MPU6050, Error> {
        // reset Board
        i2c.write(MPU6050::MPU_ADR, &[0x6B, 0b00000001])?; // use internal
                                                           // GyrX as Clock Source
        i2c.write(MPU6050::MPU_ADR, &[0x6C, 0x00])?; // Disable standby mode

        // Gyro config
        match gyr_conf {
            GyrConfig::Gyr250 => i2c.write(MPU6050::MPU_ADR, &[0x1B, 0b00000000])?,
            GyrConfig::Gyr500 => i2c.write(MPU6050::MPU_ADR, &[0x1B, 0b00001000])?,
            GyrConfig::Gyr1000 => i2c.write(MPU6050::MPU_ADR, &[0x1B, 0b00100000])?,
            GyrConfig::Gyr2000 => i2c.write(MPU6050::MPU_ADR, &[0x1B, 0b00110000])?,
        }

        // Accelerometer config
        match acc_conf {
            AccConfig::Acc2g => i2c.write(MPU6050::MPU_ADR, &[0x1C, 0b00000000])?,
            AccConfig::Acc4g => i2c.write(MPU6050::MPU_ADR, &[0x1C, 0b00001000])?,
            AccConfig::Acc8g => i2c.write(MPU6050::MPU_ADR, &[0x1C, 0b00010000])?,
            AccConfig::Acc16g => i2c.write(MPU6050::MPU_ADR, &[0x1C, 0b00110000])?,
        }

        Ok(MPU6050 {
            acc_conf,
            gyr_conf,
            raw_data: [0; 14],
            data: [0; 7],
            offsets: [0; 7],
        })
    }

    fn convert_data(&mut self) {
        for n in 0..self.data.len() {
            self.data[n] = i16::from_be_bytes([self.raw_data[n * 2], self.raw_data[n * 2 + 1]]);
        }
    }

    pub fn read_data(&mut self, i2c: &mut I2c) -> Result<(), Error> {
        i2c.write_read(
            MPU6050::MPU_ADR,
            &[MPU6050::SENSORS_START],
            &mut self.raw_data,
        )?;
        self.convert_data();
        Ok(())
    }

    pub fn calibrate(&mut self, i2c: &mut I2c) -> Result<(), Error> {
        let temp_sensor = 3;
        let mut offsets: [i32; 7] = [0; 7];
        for _ in 0..200 {
            self.read_data(i2c)?;
            for (n, offset) in offsets.iter_mut().enumerate() {
                if n == temp_sensor {
                    // skip Temp calibration
                    continue;
                }
                *offset += self.data[n] as i32;
            }
            arduino_hal::delay_ms(1);
        }

        for (n, val) in offsets.iter_mut().enumerate() {
            if n == temp_sensor {
                continue;
            }
            self.offsets[n] = (*val / 200) as i16;
        }
        self.offsets[2] -= 16384;

        Ok(())
    }

    pub fn get_data(&self, sensor: Sensor) -> f32 {
        let acc_divider;
        let gyr_divider;

        match self.acc_conf {
            AccConfig::Acc2g => acc_divider = 16384.0,
            AccConfig::Acc4g => acc_divider = 8192.0,
            AccConfig::Acc8g => acc_divider = 4096.0,
            AccConfig::Acc16g => acc_divider = 2048.0,
        }

        match self.gyr_conf {
            GyrConfig::Gyr250 => gyr_divider = 131.0,
            GyrConfig::Gyr500 => gyr_divider = 65.5,
            GyrConfig::Gyr1000 => gyr_divider = 32.8,
            GyrConfig::Gyr2000 => gyr_divider = 16.4,
        }

        match sensor {
            Sensor::AccX => return (self.data[0] - self.offsets[0]) as f32 / acc_divider,
            Sensor::AccY => return (self.data[1] - self.offsets[1]) as f32 / acc_divider,
            Sensor::AccZ => return (self.data[2] - self.offsets[2]) as f32 / acc_divider,
            Sensor::Temp => return self.data[3] as f32 / 340.0 + 36.53,
            Sensor::GyrX => return (self.data[4] - self.offsets[4]) as f32 / gyr_divider,
            Sensor::GyrY => return (self.data[5] - self.offsets[5]) as f32 / gyr_divider,
            Sensor::GyrZ => return (self.data[6] - self.offsets[6]) as f32 / gyr_divider,
            Sensor::AccXRaw => return self.data[0] as f32,
        }
    }

    pub fn print(&self, serial: &mut arduino_hal::hal::usart::Usart0<arduino_hal::DefaultClock>) {
        // Acc X
        let mut data = self.get_data(Sensor::AccX);
        let mut data_sym = "";
        if data < 0.0 {
            data_sym = "-";
            data = data * -1.0;
        }
        ufmt::uwriteln!(serial, "Acc X: {}{}", data_sym, uFmt_f32::Two(data)).unwrap_infallible();

        // Acc Y
        let mut data = self.get_data(Sensor::AccY);
        let mut data_sym = "";
        if data < 0.0 {
            data_sym = "-";
            data = data * -1.0;
        }
        ufmt::uwriteln!(serial, "Acc Y: {}{}", data_sym, uFmt_f32::Two(data)).unwrap_infallible();

        // Acc Z
        let mut data = self.get_data(Sensor::AccZ);
        let mut data_sym = "";
        if data < 0.0 {
            data_sym = "-";
            data = data * -1.0;
        }
        ufmt::uwriteln!(serial, "Acc Z: {}{}", data_sym, uFmt_f32::Two(data)).unwrap_infallible();

        // Temp
        let mut data = self.get_data(Sensor::Temp);
        let mut data_sym = "";
        if data < 0.0 {
            data_sym = "-";
            data = data * -1.0;
        }
        ufmt::uwriteln!(serial, "Temp: {}{}", data_sym, uFmt_f32::Two(data)).unwrap_infallible();

        // Gyr X
        let mut data = self.get_data(Sensor::GyrX);
        let mut data_sym = "";
        if data < 0.0 {
            data_sym = "-";
            data = data * -1.0;
        }
        ufmt::uwriteln!(serial, "Gyr X {}{}", data_sym, uFmt_f32::Two(data)).unwrap_infallible();

        // Gyr Y
        let mut data = self.get_data(Sensor::GyrY);
        let mut data_sym = "";
        if data < 0.0 {
            data_sym = "-";
            data = data * -1.0;
        }
        ufmt::uwriteln!(serial, "Gyr Y {}{}", data_sym, uFmt_f32::Two(data)).unwrap_infallible();

        // Gyr Z
        let mut data = self.get_data(Sensor::GyrZ);
        let mut data_sym = "";
        if data < 0.0 {
            data_sym = "-";
            data = data * -1.0;
        }
        ufmt::uwriteln!(serial, "Gyr Z {}{}", data_sym, uFmt_f32::Two(data)).unwrap_infallible();

        ufmt::uwriteln!(serial, "_______").unwrap_infallible();
    }
}
