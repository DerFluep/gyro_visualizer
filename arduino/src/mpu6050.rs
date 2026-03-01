use core::f32::consts::PI;

use arduino_hal::{i2c::Error, prelude::*, I2c};
use micromath::F32Ext;
use ufmt::{uWrite, uwriteln};
use ufmt_float::uFmt_f32;

use crate::millis::millis;

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
pub enum Measurements {
    AccX,
    AccY,
    AccZ,
    Temp,
    GyrX,
    GyrY,
    GyrZ,
    AccRoll,
    AccPitch,
    GyrXRot,
    GyrYRot,
    GyrZRot,
    CompRoll,
    CompPitch,
}

/// |         |   ACCELEROMETER    |           GYROSCOPE              |
/// |DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate |
/// |---------|-----------|--------|-----------|--------|-------------|
/// |0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz        |
/// |1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz        |
/// |2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz        |
/// |3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz        |
/// |4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz        |
/// |5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz        |
/// |6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz        |
#[allow(dead_code)]
pub enum Dlpf {
    Zero,
    One,
    Two,
    Three,
    Four,
    Five,
    Six,
}

pub struct MPU6050 {
    gyr_conf: GyrConfig,
    acc_conf: AccConfig,
    raw_data: [u8; 14],
    acc_x: f32,
    acc_y: f32,
    acc_z: f32,
    temp: f32,
    gyr_x: f32,
    gyr_y: f32,
    gyr_z: f32,
    acc_x_off: i16,
    acc_y_off: i16,
    acc_z_off: i16,
    gyr_x_off: i16,
    gyr_y_off: i16,
    gyr_z_off: i16,
    acc_roll: f32,
    acc_pitch: f32,
    gyr_x_rot: f32,
    gyr_y_rot: f32,
    gyr_z_rot: f32,
    comp_roll: f32,
    comp_pitch: f32,
    prev_time: u32,
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

    pub fn new(
        i2c: &mut I2c,
        gyr_conf: GyrConfig,
        acc_conf: AccConfig,
        dlpf: Dlpf,
    ) -> Result<MPU6050, Error> {
        // reset Board
        i2c.write(MPU6050::MPU_ADR, &[0x6B, 0b00000001])?; // use internal
                                                           // GyrX as Clock Source
        i2c.write(MPU6050::MPU_ADR, &[0x6C, 0x00])?; // Disable standby mode

        match dlpf {
            Dlpf::Zero => i2c.write(MPU6050::MPU_ADR, &[0x1A, 0b00000000])?,
            Dlpf::One => i2c.write(MPU6050::MPU_ADR, &[0x1A, 0b00000001])?,
            Dlpf::Two => i2c.write(MPU6050::MPU_ADR, &[0x1A, 0b00000010])?,
            Dlpf::Three => i2c.write(MPU6050::MPU_ADR, &[0x1A, 0b00000011])?,
            Dlpf::Four => i2c.write(MPU6050::MPU_ADR, &[0x1A, 0b00000100])?,
            Dlpf::Five => i2c.write(MPU6050::MPU_ADR, &[0x1A, 0b00000101])?,
            Dlpf::Six => i2c.write(MPU6050::MPU_ADR, &[0x1A, 0b00000110])?,
        }

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
            acc_x: 0.0,
            acc_y: 0.0,
            acc_z: 0.0,
            temp: 0.0,
            gyr_x: 0.0,
            gyr_y: 0.0,
            gyr_z: 0.0,
            acc_x_off: 0,
            acc_y_off: 0,
            acc_z_off: 0,
            gyr_x_off: 0,
            gyr_y_off: 0,
            gyr_z_off: 0,
            acc_roll: 0.0,
            acc_pitch: 0.0,
            gyr_x_rot: 0.0,
            gyr_y_rot: 0.0,
            gyr_z_rot: 0.0,
            comp_roll: 0.0,
            comp_pitch: 0.0,
            prev_time: millis(),
        })
    }

    pub fn read_data(&mut self, i2c: &mut I2c) -> Result<(), Error> {
        match i2c.write_read(
            MPU6050::MPU_ADR,
            &[MPU6050::SENSORS_START],
            &mut self.raw_data,
        ) {
            Ok(_) => {}
            Err(e) => {
                self.prev_time = millis();
                return Err(e);
            }
        }
        self.convert_data();
        Ok(())
    }

    fn convert_data(&mut self) {
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

        self.acc_x = i16::from_be_bytes([self.raw_data[0], self.raw_data[1]])
            .saturating_sub(self.acc_x_off) as f32
            / acc_divider;

        self.acc_y = i16::from_be_bytes([self.raw_data[2], self.raw_data[3]])
            .saturating_sub(self.acc_y_off) as f32
            / acc_divider;

        self.acc_z = i16::from_be_bytes([self.raw_data[4], self.raw_data[5]])
            .saturating_sub(self.acc_z_off) as f32
            / acc_divider;

        self.temp = i16::from_be_bytes([self.raw_data[6], self.raw_data[7]]) as f32 / 340.0 + 36.53;

        self.gyr_x = i16::from_be_bytes([self.raw_data[8], self.raw_data[9]])
            .saturating_sub(self.gyr_x_off) as f32
            / gyr_divider;

        self.gyr_y = i16::from_be_bytes([self.raw_data[10], self.raw_data[11]])
            .saturating_sub(self.gyr_y_off) as f32
            / gyr_divider;

        self.gyr_z = i16::from_be_bytes([self.raw_data[12], self.raw_data[13]])
            .saturating_sub(self.gyr_z_off) as f32
            / gyr_divider;

        let now = millis();
        let delta_t = (now as f32 - self.prev_time as f32) / 1000.0;
        self.gyr_x_rot += self.gyr_x * delta_t;
        self.gyr_y_rot += self.gyr_y * delta_t;
        self.gyr_z_rot += self.gyr_z * delta_t;

        self.acc_roll = self.acc_y.atan2(self.acc_z) * 180.0 / PI;
        self.acc_pitch = -self
            .acc_x
            .atan2((self.acc_y.powi(2) + self.acc_z.powi(2)).sqrt())
            * 180.0
            / PI;

        let alpha = 0.98;
        self.comp_roll =
            alpha * (self.comp_roll + self.gyr_x * delta_t) + (1.0 - alpha) * self.acc_roll;
        self.comp_pitch =
            alpha * (self.comp_pitch + self.gyr_y * delta_t) + (1.0 - alpha) * self.acc_pitch;
        self.prev_time = now;
    }

    pub fn calibrate(&mut self, i2c: &mut I2c) -> Result<(), Error> {
        let mut acc_x_off = 0;
        let mut acc_y_off = 0;
        let mut acc_z_off = 0;
        let mut gyr_x_off = 0;
        let mut gyr_y_off = 0;
        let mut gyr_z_off = 0;

        let repeats = 1000;

        for _ in 0..repeats {
            i2c.write_read(
                MPU6050::MPU_ADR,
                &[MPU6050::SENSORS_START],
                &mut self.raw_data,
            )?;

            acc_x_off += i16::from_be_bytes([self.raw_data[0], self.raw_data[1]]) as i32;
            acc_y_off += i16::from_be_bytes([self.raw_data[2], self.raw_data[3]]) as i32;
            acc_z_off += i16::from_be_bytes([self.raw_data[4], self.raw_data[5]]) as i32;
            gyr_x_off += i16::from_be_bytes([self.raw_data[8], self.raw_data[9]]) as i32;
            gyr_y_off += i16::from_be_bytes([self.raw_data[10], self.raw_data[11]]) as i32;
            gyr_z_off += i16::from_be_bytes([self.raw_data[12], self.raw_data[13]]) as i32;

            arduino_hal::delay_ms(1);
        }

        self.acc_x_off = (acc_x_off / repeats) as i16;
        self.acc_y_off = (acc_y_off / repeats) as i16;
        self.acc_z_off = (acc_z_off / repeats - 16384) as i16;
        self.gyr_x_off = (gyr_x_off / repeats) as i16;
        self.gyr_y_off = (gyr_y_off / repeats) as i16;
        self.gyr_z_off = (gyr_z_off / repeats) as i16;

        Ok(())
    }

    pub fn get_data(&self, measurement: Measurements) -> f32 {
        match measurement {
            Measurements::AccX => return self.acc_x,
            Measurements::AccY => return self.acc_y,
            Measurements::AccZ => return self.acc_z,
            Measurements::Temp => return self.temp,
            Measurements::GyrX => return self.gyr_x,
            Measurements::GyrY => return self.gyr_y,
            Measurements::GyrZ => return self.gyr_z,
            Measurements::AccRoll => return self.acc_roll,
            Measurements::AccPitch => return self.acc_pitch,
            Measurements::GyrXRot => return self.gyr_x_rot,
            Measurements::GyrYRot => return self.gyr_y_rot,
            Measurements::GyrZRot => return self.gyr_z_rot,
            Measurements::CompRoll => return self.comp_roll,
            Measurements::CompPitch => return self.comp_pitch,
        }
    }

    #[allow(dead_code)]
    pub fn print<W: uWrite<Error = ::core::convert::Infallible>>(&self, serial: &mut W) {
        // Acc X
        let mut data = self.get_data(Measurements::AccX);
        let mut data_sym = "";
        if data < 0.0 {
            data_sym = "-";
            data = data * -1.0;
        }
        uwriteln!(serial, "Acc X: {}{}", data_sym, uFmt_f32::Two(data)).unwrap_infallible();

        // Acc Y
        let mut data = self.get_data(Measurements::AccY);
        let mut data_sym = "";
        if data < 0.0 {
            data_sym = "-";
            data = data * -1.0;
        }
        uwriteln!(serial, "Acc Y: {}{}", data_sym, uFmt_f32::Two(data)).unwrap_infallible();

        // Acc Z
        let mut data = self.get_data(Measurements::AccZ);
        let mut data_sym = "";
        if data < 0.0 {
            data_sym = "-";
            data = data * -1.0;
        }
        uwriteln!(serial, "Acc Z: {}{}", data_sym, uFmt_f32::Two(data)).unwrap_infallible();

        // Temp
        let mut data = self.get_data(Measurements::Temp);
        let mut data_sym = "";
        if data < 0.0 {
            data_sym = "-";
            data = data * -1.0;
        }
        uwriteln!(serial, "Temp: {}{}", data_sym, uFmt_f32::Two(data)).unwrap_infallible();

        // Gyr X
        let mut data = self.get_data(Measurements::GyrX);
        let mut data_sym = "";
        if data < 0.0 {
            data_sym = "-";
            data = data * -1.0;
        }
        uwriteln!(serial, "Gyr X {}{}", data_sym, uFmt_f32::Two(data)).unwrap_infallible();

        // Gyr Y
        let mut data = self.get_data(Measurements::GyrY);
        let mut data_sym = "";
        if data < 0.0 {
            data_sym = "-";
            data = data * -1.0;
        }
        uwriteln!(serial, "Gyr Y {}{}", data_sym, uFmt_f32::Two(data)).unwrap_infallible();

        // Gyr Z
        let mut data = self.get_data(Measurements::GyrZ);
        let mut data_sym = "";
        if data < 0.0 {
            data_sym = "-";
            data = data * -1.0;
        }
        uwriteln!(serial, "Gyr Z {}{}", data_sym, uFmt_f32::Two(data)).unwrap_infallible();

        // Gyr X Rot
        let mut data = self.get_data(Measurements::GyrXRot);
        let mut data_sym = "";
        if data < 0.0 {
            data_sym = "-";
            data = data * -1.0;
        }
        uwriteln!(serial, "Gyr X Rotation {}{}", data_sym, uFmt_f32::Two(data)).unwrap_infallible();

        // Gyr Y Rot
        let mut data = self.get_data(Measurements::GyrYRot);
        let mut data_sym = "";
        if data < 0.0 {
            data_sym = "-";
            data = data * -1.0;
        }
        uwriteln!(serial, "Gyr Y Rotation {}{}", data_sym, uFmt_f32::Two(data)).unwrap_infallible();

        // Gyr Z Rot
        let mut data = self.get_data(Measurements::GyrZRot);
        let mut data_sym = "";
        if data < 0.0 {
            data_sym = "-";
            data = data * -1.0;
        }
        uwriteln!(serial, "Gyr Z Rotation {}{}", data_sym, uFmt_f32::Two(data)).unwrap_infallible();

        // acc roll
        let mut data = self.acc_roll;
        let mut data_sym = "";
        if data < 0.0 {
            data_sym = "-";
            data = data * -1.0;
        }
        uwriteln!(serial, "Acc roll {}{}", data_sym, uFmt_f32::Two(data)).unwrap_infallible();

        // acc pitch
        let mut data = self.acc_pitch;
        let mut data_sym = "";
        if data < 0.0 {
            data_sym = "-";
            data = data * -1.0;
        }
        uwriteln!(serial, "Acc pitch {}{}", data_sym, uFmt_f32::Two(data)).unwrap_infallible();

        // Combined Roll
        let mut data = self.comp_roll;
        let mut data_sym = "";
        if data < 0.0 {
            data_sym = "-";
            data = data * -1.0;
        }
        uwriteln!(serial, "Combined Roll {}{}", data_sym, uFmt_f32::Two(data)).unwrap_infallible();

        // Combined Pitch
        let mut data = self.comp_pitch;
        let mut data_sym = "";
        if data < 0.0 {
            data_sym = "-";
            data = data * -1.0;
        }
        uwriteln!(serial, "Combined Pitch {}{}", data_sym, uFmt_f32::Two(data)).unwrap_infallible();

        uwriteln!(serial, "_______").unwrap_infallible();
    }
}
