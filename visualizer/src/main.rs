use std::time::Duration;

fn main() {
    let mut port = serialport::new("/dev/ttyACM0", 9600)
        .timeout(Duration::from_millis(300))
        .open()
        .expect("Failed to open port");
    port.set_baud_rate(57600).unwrap();
    ::std::thread::sleep(Duration::from_millis(5000));

    let mut buf = [0u8; 8];
    let mut start = [0u8; 1];

    loop {
        'checking: loop {
            port.read_exact(&mut start).unwrap();
            if start[0] == 0 {
                break 'checking;
            }
        }
        port.read_exact(&mut buf).unwrap();
        let roll = f32::from_be_bytes(buf[0..4].try_into().unwrap());
        let pitch = f32::from_be_bytes(buf[4..8].try_into().unwrap());
        println!("Roll: {}", roll);
        println!("Pitch: {}", pitch);
        println!("______");
    }
}
