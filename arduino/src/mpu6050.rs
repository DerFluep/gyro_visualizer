use arduino_hal::{prelude::*, I2c};

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
    i2c: I2c,
    gyr_conf: GyrConfig,
    acc_conf: AccConfig,
    raw_data: [u8; 14],
    data: [i16; 7],
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

    pub fn new(mut i2c: I2c, gyr_conf: GyrConfig, acc_conf: AccConfig) -> MPU6050 {
        // reset Board
        i2c.write(MPU6050::MPU_ADR, &[0x6B, 0b00000001]).unwrap(); // use internal
                                                                   // GyrX as Clock Source
        i2c.write(MPU6050::MPU_ADR, &[0x6C, 0x00]).unwrap(); // Disable standby mode

        // Gyro config
        match gyr_conf {
            GyrConfig::Gyr250 => i2c.write(MPU6050::MPU_ADR, &[0x1B, 0b00000000]).unwrap(),
            GyrConfig::Gyr500 => i2c.write(MPU6050::MPU_ADR, &[0x1B, 0b00001000]).unwrap(),
            GyrConfig::Gyr1000 => i2c.write(MPU6050::MPU_ADR, &[0x1B, 0b00100000]).unwrap(),
            GyrConfig::Gyr2000 => i2c.write(MPU6050::MPU_ADR, &[0x1B, 0b00110000]).unwrap(),
        }

        // Accelerometer config
        match acc_conf {
            AccConfig::Acc2g => i2c.write(MPU6050::MPU_ADR, &[0x1C, 0b00000000]).unwrap(),
            AccConfig::Acc4g => i2c.write(MPU6050::MPU_ADR, &[0x1C, 0b00001000]).unwrap(),
            AccConfig::Acc8g => i2c.write(MPU6050::MPU_ADR, &[0x1C, 0b00010000]).unwrap(),
            AccConfig::Acc16g => i2c.write(MPU6050::MPU_ADR, &[0x1C, 0b00110000]).unwrap(),
        }

        MPU6050 {
            i2c,
            acc_conf,
            gyr_conf,
            raw_data: [0; 14],
            data: [0; 7],
        }
    }

    fn convert_data(&mut self) {
        for n in 0..self.data.len() {
            // self.data[n] = ((self.raw_data[n * 2] as i16) << 8) | self.raw_data[n * 2 + 1] as i16;
            self.data[n] = i16::from_be_bytes([self.raw_data[n * 2], self.raw_data[n * 2 + 1]]);
        }
    }

    pub fn read_data(&mut self) {
        self.i2c
            .write_read(
                MPU6050::MPU_ADR,
                &[MPU6050::SENSORS_START],
                &mut self.raw_data,
            )
            .unwrap();
        self.convert_data();
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
            Sensor::AccX => return self.data[0] as f32 / acc_divider,
            Sensor::AccY => return self.data[1] as f32 / acc_divider,
            Sensor::AccZ => return self.data[2] as f32 / acc_divider,
            Sensor::Temp => return self.data[3] as f32 / 340.0 + 36.53,
            Sensor::GyrX => return self.data[4] as f32 / gyr_divider,
            Sensor::GyrY => return self.data[5] as f32 / gyr_divider,
            Sensor::GyrZ => return self.data[6] as f32 / gyr_divider,
            Sensor::AccXRaw => return self.data[0] as f32,
        }
    }
}
