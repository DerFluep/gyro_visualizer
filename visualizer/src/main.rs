use std::time::Duration;

fn main() {
    let mut port = serialport::new("/dev/ttyACM0", 9600)
        .timeout(Duration::from_millis(250))
        .open()
        .expect("Failed to open port");
    port.set_baud_rate(9600).unwrap();
    ::std::thread::sleep(Duration::from_millis(5000));

    let mut buf = [0u8; 6];
    let mut start = [0u8; 1];

    let mut acc_x: i16;
    let mut acc_y: i16;
    let mut acc_z: i16;

    loop {
        'checking: loop {
            port.read_exact(&mut start).unwrap();
            if start[0] == 0 {
                break 'checking;
            }
        }
        port.read_exact(&mut buf).unwrap();
        acc_x = i16::from_be_bytes(buf[0..2].try_into().unwrap());
        acc_y = i16::from_be_bytes(buf[2..4].try_into().unwrap());
        acc_z = i16::from_be_bytes(buf[4..6].try_into().unwrap());
        println!("{}", acc_x as f32 / 16384.0);
        println!("{}", acc_y as f32 / 16384.0);
        println!("{}", acc_z as f32 / 16384.0);
        println!("______");
    }
}
