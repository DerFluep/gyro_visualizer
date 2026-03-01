use std::time::Duration;

fn main() {
    let mut port = serialport::new("/dev/ttyACM0", 57600)
        .timeout(Duration::from_millis(250))
        .open()
        .expect("Failed to open port");

    let mut serial_buf = [0u8; 1];
    let mut buf = [0u8; 8];
    let header = [0xAA, 0xBB, 0xCC, 0xDD];
    let mut sync_state = 0;

    loop {
        match port.read_exact(&mut serial_buf) {
            Ok(_) => {}
            Err(e) => {
                println!("Can't read serial: {}", e);
                continue;
            }
        }
        if serial_buf[0] == header[sync_state] {
            sync_state += 1;
            if sync_state == 4 {
                match port.read_exact(&mut buf) {
                    Ok(_) => {}
                    Err(e) => {
                        println!("Error reading floats: {}", e);
                    }
                }
                let roll = f32::from_be_bytes(buf[0..4].try_into().unwrap());
                let pitch = f32::from_be_bytes(buf[4..8].try_into().unwrap());
                println!("Roll: {}", roll);
                println!("Pitch: {}", pitch);
                println!("______");
                sync_state = 0;
            }
        }
    }
}
