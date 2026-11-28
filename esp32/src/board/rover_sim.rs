//! The `waveshare-ros-driver` board's link to a physics simulation (`--board
//! waveshare-ros-driver,sim=HOST:PORT`): the board connects to HOST:PORT over TCP, retrying
//! while nobody listens, and exchanges one JSON object per line in wall-clock time (the run is
//! `--realtime`-paced when a host drives it, as with `--uart-tcp`).
//!
//! Board to simulation, every 10 ms: `{"duty":[l,r],"speed":[l,r],"travel":[l,r],"pt":[pan,tilt],"led":[io4,io5]}`,
//! the signed H-bridge duty (-1..1) per side, the wheel speeds (m/s) and travel (m) the encoders
//! report, and the pan-tilt gimbal's present angles in degrees from its servos' shafts, in the
//! firmware's convention (`ros_driver::GimbalShared`: T:133's X and Y, T:1001's pan and tilt; 0, 0
//! at the servos' middle, pan = (pos - 2047) * 360 / 4095, tilt = (2047 - pos) * 360 / 4095),
//! and the two LED lights' brightness 0..1 (`led`: IO4 by the depth camera, IO5 by the USB camera,
//! as the firmware's T:132 sets their PWM).
//! Simulation to board, any subset of the keys in one line:
//! - `"speed":[l,r]`: wheel surface speeds in m/s; the encoders follow them instead of the built-in
//!   motor model while the link is up.
//! - `"q":[w,x,y,z]`, `"w":[x,y,z]`, `"a":[x,y,z]`, `"mag":[x,y,z]`: the IMU's pose (body to world),
//!   body angular rate (rad/s), world linear acceleration without gravity (m/s^2) and the world
//!   magnetic field (uT), as `Icm20948Input` takes them.
//! - `"bat":[volts,amps]`: what the INA219 measures.
//!
//! A line that is not UTF-8 JSON is ignored, and one longer than `LINE_MAX` (64 KiB) is dropped
//! whole, with one warning, the board reading on from the next newline; within one, a key whose value is not an array of finite
//! numbers of the right length is ignored, and so are keys the board does not know. When the
//! simulation disconnects the board returns to its own motor model and the link waits for the next.
use std::io::{BufRead, BufReader, Read, Write};
use std::net::TcpStream;
use std::time::Duration;
use esp_soc::devices::{Icm20948Input, Ina219Shared};
use esp_soc::json::{parse_json, Json};
use super::ros_driver::{DriveShared, GimbalShared};

/// The longest line taken from the simulation: one of this protocol's is well under 1 KiB, and a
/// peer that never sends a newline must not grow the emulator's memory (#388's socket board caps
/// its lines the same way).
pub const LINE_MAX: usize = 64 << 10;

/// Hands each line of `r` (without its newline) to `each` until the stream ends or fails. A line
/// longer than `max` bytes is dropped whole: the reader keeps at most `max` bytes, skips to the
/// next newline and goes on from there, and the first such line is reported once. Returns how
/// many lines were dropped.
pub fn read_lines(r: impl Read, max: usize, mut each: impl FnMut(&[u8])) -> usize {
    let mut r = BufReader::new(r);
    let (mut line, mut skipping, mut dropped) = (Vec::new(), false, 0);
    loop {
        let buf = match r.fill_buf() {
            Ok([]) => return dropped,
            Ok(b) => b,
            Err(e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
            Err(_) => return dropped,
        };
        let end = buf.iter().position(|&b| b == b'\n');
        let chunk = &buf[..end.unwrap_or(buf.len())];
        if !skipping && line.len() + chunk.len() > max {
            skipping = true;
            line.clear();
            if dropped == 0 { eprintln!("[emu] ros-driver: dropped a line from the simulation longer than {max} bytes (reported once)"); }
            dropped += 1;
        }
        if !skipping { line.extend_from_slice(chunk); }
        let used = end.map_or(buf.len(), |e| e + 1);
        r.consume(used);
        if end.is_some() {
            if !skipping { each(&line); }
            line.clear();
            skipping = false;
        }
    }
}

/// The numbers of `"key":[...]` in a parsed line, if it holds exactly `n` finite ones.
pub fn numbers(line: &Json, key: &str, n: usize) -> Option<Vec<f64>> {
    let v: Vec<f64> = line.get(key)?.as_arr()?.iter().map(Json::as_f64).collect::<Option<_>>()?;
    (v.len() == n && v.iter().all(|x| x.is_finite())).then_some(v)
}

/// Applies one line from the simulation to the board's shared state. A line that is not a JSON
/// object is ignored whole; in one that is, each key whose value is not an array of `n` finite
/// numbers is ignored.
pub fn apply(line: &str, drive: &DriveShared, imu: &Icm20948Input, battery: &Ina219Shared) {
    let Ok(j) = parse_json(line) else { return };
    if let Some(s) = numbers(&j, "speed", 2) {
        let mut d = drive.lock().unwrap_or_else(|e| e.into_inner());
        d.speed_mps = [s[0].clamp(-10.0, 10.0), s[1].clamp(-10.0, 10.0)];
        d.external = true;
    }
    if let Some(q) = numbers(&j, "q", 4) {
        let n = q.iter().map(|x| x * x).sum::<f64>().sqrt();
        if n > 1e-9 { imu.set_orientation([q[0] / n, q[1] / n, q[2] / n, q[3] / n]); }
    }
    if let Some(w) = numbers(&j, "w", 3) { imu.set_angular_rate([w[0], w[1], w[2]]); }
    if let Some(a) = numbers(&j, "a", 3) { imu.set_linear_accel([a[0], a[1], a[2]]); }
    if let Some(m) = numbers(&j, "mag", 3) { imu.set_mag_field([m[0], m[1], m[2]]); }
    if let Some(b) = numbers(&j, "bat", 2) {
        let mut i = battery.lock().unwrap_or_else(|e| e.into_inner());
        i.bus_v = b[0].clamp(0.0, 32.0);
        i.current_a = b[1].clamp(-50.0, 50.0);
    }
}

/// The board's line to the simulation.
pub fn line(d: &super::ros_driver::Drive, pt: [f64; 2]) -> String {
    format!("{{\"duty\":[{:.4},{:.4}],\"speed\":[{:.4},{:.4}],\"travel\":[{:.5},{:.5}],\"pt\":[{:.2},{:.2}],\"led\":[{:.3},{:.3}]}}\n",
            d.duty[0], d.duty[1], d.speed_mps[0], d.speed_mps[1], d.travel_m[0], d.travel_m[1], pt[0], pt[1], d.lights[0], d.lights[1])
}

/// Starts the link thread; it lives as long as the process.
pub fn spawn(addr: String, drive: DriveShared, imu: Icm20948Input, battery: Ina219Shared, gimbal: GimbalShared) {
    std::thread::Builder::new().name("rover-sim".into()).spawn(move || {
        let mut said = false;
        loop {
            let Ok(s) = TcpStream::connect(&addr) else {
                if !said { eprintln!("[emu] ros-driver: waiting for the simulation at {addr}"); said = true; }
                std::thread::sleep(Duration::from_millis(500));
                continue;
            };
            eprintln!("[emu] ros-driver: simulation connected at {addr}");
            let _ = s.set_nodelay(true);
            let Ok(reader) = s.try_clone() else { continue };
            let (d2, i2, b2) = (drive.clone(), imu.clone(), battery.clone());
            let rx = std::thread::spawn(move || {
                read_lines(reader, LINE_MAX, |l| if let Ok(l) = std::str::from_utf8(l) { apply(l, &d2, &i2, &b2) });
            });
            let mut w = s;
            while !rx.is_finished() {
                let d = *drive.lock().unwrap_or_else(|e| e.into_inner());
                let msg = line(&d, *gimbal.lock().unwrap_or_else(|e| e.into_inner()));
                if w.write_all(msg.as_bytes()).is_err() { break; }
                std::thread::sleep(Duration::from_millis(10));
            }
            let _ = w.shutdown(std::net::Shutdown::Both);
            let _ = rx.join();
            drive.lock().unwrap_or_else(|e| e.into_inner()).external = false;
            eprintln!("[emu] ros-driver: simulation disconnected");
            said = false;
        }
    }).expect("spawn the rover-sim thread");
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};
    use esp_soc::devices::Ina219Input;

    #[test]
    fn lines_set_speeds_pose_and_battery_and_bad_values_are_ignored() {
        let drive: DriveShared = Default::default();
        let imu = Icm20948Input::default();
        let bat = Arc::new(Mutex::new(Ina219Input { bus_v: 11.8, current_a: 0.25, shunt_ohm: 0.01 }));
        apply(r#"{"speed":[0.25,-0.5],"q":[0,0,0,2],"w":[0,0,1.5],"bat":[11.1, 2.5]}"#, &drive, &imu, &bat);
        let d = *drive.lock().unwrap();
        assert!(d.external && d.speed_mps == [0.25, -0.5]);
        let p = imu.get();
        assert_eq!((p.orientation, p.angular_rate), ([0.0, 0.0, 0.0, 1.0], [0.0, 0.0, 1.5]));
        let b = *bat.lock().unwrap();
        assert_eq!((b.bus_v, b.current_a), (11.1, 2.5));
        // wrong lengths, non-numbers, missing brackets: no change, no panic
        apply(r#"{"speed":[1],"w":[0,0,"x"],"a":[1,0],"q":[0,0,0,0],"bat":5}"#, &drive, &imu, &bat);
        // not JSON (an out-of-range number, a cut line, a key inside a string): the whole line is ignored
        apply(r#"{"speed":[2,2],"a":[1e999,0,0]}"#, &drive, &imu, &bat);
        apply(r#"{"speed":[2,2],"w":[1,1,1]"#, &drive, &imu, &bat);
        apply(r#"{"src":"\"speed\":[3,3]"}"#, &drive, &imu, &bat);
        assert_eq!(drive.lock().unwrap().speed_mps, [0.25, -0.5]);
        assert_eq!(imu.get().angular_rate, [0.0, 0.0, 1.5]);
        assert_eq!(imu.get().linear_accel, [0.0; 3]);
        assert_eq!(imu.get().orientation, [0.0, 0.0, 0.0, 1.0]);
        // a key's name inside another key's string value does not hide the key itself
        apply(r#"{"src":"a","a" : [ 1, 2.5 ,-3 ]}"#, &drive, &imu, &bat);
        assert_eq!(imu.get().linear_accel, [1.0, 2.5, -3.0]);
    }

    #[test]
    fn an_overlong_line_is_dropped_whole_and_reading_goes_on() {
        // 8 MiB without a newline, then a good line: only the good line arrives, and the reader
        // never holds more than the cap (it would otherwise grow one 8 MiB line)
        let flood = std::io::repeat(b'x').take(8 << 20);
        let rest = std::io::Cursor::new(&b"\n{\"speed\":[1,1]}\nshort\n\xff\xfe\n"[..]);
        let mut got = Vec::new();
        let dropped = read_lines(flood.chain(rest), LINE_MAX, |l| got.push(l.to_vec()));
        assert_eq!(dropped, 1);
        assert_eq!(got, [b"{\"speed\":[1,1]}".to_vec(), b"short".to_vec(), vec![0xff, 0xfe]]);
        // exactly the cap is kept; one byte more is not
        let mut sizes = Vec::new();
        let text = [vec![b'a'; 16], vec![b'\n'], vec![b'b'; 17], vec![b'\n'], vec![b'c'; 3]].concat();
        assert_eq!(read_lines(&text[..], 16, |l| sizes.push(l.len())), 1);
        assert_eq!(sizes, [16], "the unterminated tail at the end of the stream is not a line");
        // a non-UTF-8 line applies nothing and does not end the link
        let drive: DriveShared = Default::default();
        let (imu, bat) = (Icm20948Input::default(), Arc::new(Mutex::new(Ina219Input { bus_v: 11.8, current_a: 0.25, shunt_ohm: 0.01 })));
        read_lines(&b"\xff{\"speed\":[2,2]}\n{\"speed\":[1,1]}\n"[..], LINE_MAX, |l| if let Ok(l) = std::str::from_utf8(l) { apply(l, &drive, &imu, &bat) });
        assert_eq!(drive.lock().unwrap().speed_mps, [1.0, 1.0]);
    }

    #[test]
    fn the_board_line_carries_wheels_gimbal_and_lights() {
        let d = super::super::ros_driver::Drive { lights: [0.5, 1.0], ..Default::default() };
        let l = line(&d, [44.923, -19.956]);
        assert!(l.ends_with("}\n"));
        let j = parse_json(&l).unwrap();
        assert_eq!(numbers(&j, "led", 2), Some(vec![0.5, 1.0]));
        assert_eq!(numbers(&j, "pt", 2), Some(vec![44.92, -19.96]));
        assert_eq!(numbers(&j, "travel", 2), Some(vec![0.0, 0.0]));
    }
}
