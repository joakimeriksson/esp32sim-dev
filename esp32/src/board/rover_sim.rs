//! The `waveshare-ros-driver` board's link to a physics simulation (`--board
//! waveshare-ros-driver,sim=HOST:PORT`): the board connects to HOST:PORT over TCP, retrying
//! while nobody listens, and exchanges one JSON object per line. The protocol is provisional: it
//! is meant to become message types on #388's socket session, whose `sync`/`quiet_until` would
//! hold the guest in lock-step with the simulation.
//!
//! Time: every line the board sends carries `"t"`, the emulated time in ns (as `--cooja` stamps
//! its messages), and the board sends one every 10 ms of emulated time (`PERIOD_NS`), boundaries
//! at multiples of 10 ms. What the simulation sends is applied when it arrives, so the guest sees
//! it at whatever emulated time that is; `sim=` therefore implies `--realtime`, which holds the
//! run to at most 1x, the rate a simulation integrating its own wall clock assumes. A host too slow
//! for real time still runs slower than 1x, and `"t"` shows the simulation by how much. Runs with
//! a simulation are not reproducible.
//!
//! Board to simulation, every 10 ms of emulated time: `{"t":ns,"duty":[l,r],"speed":[l,r],"travel":[l,r],"pt":[pan,tilt],"led":[io4,io5]}`,
//! the emulated time, the signed H-bridge duty (-1..1) per side, the wheel speeds (m/s) and travel (m) the encoders
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
//! simulation disconnects the board returns to its own motor model and an IMU at rest, keeping the
//! orientation, magnetic field and battery reading the simulation last set (`disconnected`), and
//! the link waits for the next.
use std::io::{BufRead, BufReader, Read};
use std::sync::mpsc::SyncSender;
use esp_soc::devices::{Icm20948Input, Ina219Shared};
use esp_soc::json::{parse_json, Json};
use super::ros_driver::{Drive, DriveShared};

/// The board sends a line every 10 ms of emulated time.
pub const PERIOD_NS: u64 = 10_000_000;
/// Board lines waiting for the link thread; while it is stuck on the socket, newer lines are dropped.
pub const QUEUE: usize = 64;

/// The board's end of the link: every `PERIOD_NS` of emulated time it stamps a line and queues it
/// for the link thread, so the guest never waits on the socket.
pub struct Link { tx: SyncSender<String>, hz: u64, period: u64, next: u64 }
impl Link {
    /// A link whose times are cycles of an `hz` clock, sending into `tx`.
    pub fn new(tx: SyncSender<String>, hz: u64) -> Self {
        let hz = hz.max(1);
        let period = u64::try_from(u128::from(hz) * u128::from(PERIOD_NS) / 1_000_000_000).unwrap_or(u64::MAX).max(1);
        Link { tx, hz, period, next: 0 }
    }
    /// When the next line is due (cycles).
    pub fn next(&self) -> u64 { self.next }
    /// Queues the line due by `cycle`, if one is, stamped with the latest boundary at or before it
    /// (a boundary the board was not woken for is skipped, not sent late).
    pub fn send_due(&mut self, cycle: u64, d: &Drive, pt: [f64; 2]) {
        if cycle < self.next { return; }
        let at = self.next + (cycle - self.next) / self.period * self.period;
        let t = u64::try_from(u128::from(at) * 1_000_000_000 / u128::from(self.hz)).unwrap_or(u64::MAX);
        // a full queue (the link thread stuck on the socket) or no link thread: the line is dropped
        let _ = self.tx.try_send(line(t, d, pt));
        self.next = at.saturating_add(self.period);
    }
}

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

/// The board's line to the simulation at emulated time `t_ns`.
pub fn line(t_ns: u64, d: &Drive, pt: [f64; 2]) -> String {
    format!("{{\"t\":{t_ns},\"duty\":[{:.4},{:.4}],\"speed\":[{:.4},{:.4}],\"travel\":[{:.5},{:.5}],\"pt\":[{:.2},{:.2}],\"led\":[{:.3},{:.3}]}}\n",
            d.duty[0], d.duty[1], d.speed_mps[0], d.speed_mps[1], d.travel_m[0], d.travel_m[1], pt[0], pt[1], d.lights[0], d.lights[1])
}

/// What the board keeps when the simulation goes: it falls back to what it does without one,
/// its own motor model (from the speeds the wheels had) and an IMU at rest (angular rate and linear
/// acceleration zero), but nothing moves the rover back, so the orientation, the magnetic field
/// and the battery reading stay where the simulation left them.
pub fn disconnected(drive: &DriveShared, imu: &Icm20948Input) {
    imu.set_angular_rate([0.0; 3]);
    imu.set_linear_accel([0.0; 3]);
    drive.lock().unwrap_or_else(|e| e.into_inner()).external = false;
}

/// Starts the link thread. It writes the lines the board queues on `rx` to the simulation (and
/// drops them while none is connected), applies what comes back, and ends when the board is gone.
#[cfg(not(target_arch = "wasm32"))]
pub fn spawn(addr: String, rx: std::sync::mpsc::Receiver<String>, drive: DriveShared, imu: Icm20948Input, battery: Ina219Shared) {
    use std::io::Write;
    use std::sync::mpsc::{RecvTimeoutError, TryRecvError};
    use std::time::Duration;
    std::thread::Builder::new().name("rover-sim".into()).spawn(move || {
        let mut said = false;
        loop {
            // lines from while nobody listened are stale
            loop { match rx.try_recv() { Ok(_) => {} Err(TryRecvError::Empty) => break, Err(TryRecvError::Disconnected) => return } }
            let Ok(s) = std::net::TcpStream::connect(&addr) else {
                if !said { eprintln!("[emu] ros-driver: waiting for the simulation at {addr}"); said = true; }
                std::thread::sleep(Duration::from_millis(500));
                continue;
            };
            eprintln!("[emu] ros-driver: simulation connected at {addr}");
            let _ = s.set_nodelay(true);
            let Ok(reader) = s.try_clone() else { continue };
            let (d2, i2, b2) = (drive.clone(), imu.clone(), battery.clone());
            let peer = std::thread::spawn(move || {
                read_lines(reader, LINE_MAX, |l| if let Ok(l) = std::str::from_utf8(l) { apply(l, &d2, &i2, &b2) });
            });
            let mut w = s;
            let board_gone = loop {
                if peer.is_finished() { break false; }
                match rx.recv_timeout(Duration::from_millis(100)) {
                    Ok(msg) => if w.write_all(msg.as_bytes()).is_err() { break false; },
                    Err(RecvTimeoutError::Timeout) => {}
                    Err(RecvTimeoutError::Disconnected) => break true,
                }
            };
            let _ = w.shutdown(std::net::Shutdown::Both);
            let _ = peer.join();
            disconnected(&drive, &imu);
            eprintln!("[emu] ros-driver: simulation disconnected");
            if board_gone { return; }
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
        // the battery is clamped to what an INA219 bus can carry; the field sets the magnetometer
        apply(r#"{"bat":[40,-60],"mag":[1,2,3]}"#, &drive, &imu, &bat);
        let b = *bat.lock().unwrap();
        assert_eq!((b.bus_v, b.current_a), (32.0, -50.0));
        assert_eq!(imu.get().mag_field, [1.0, 2.0, 3.0]);
        // a key's name inside another key's string value does not hide the key itself
        apply(r#"{"src":"a","a" : [ 1, 2.5 ,-3 ]}"#, &drive, &imu, &bat);
        assert_eq!(imu.get().linear_accel, [1.0, 2.5, -3.0]);
    }

    #[test]
    fn a_disconnect_stops_the_motion_and_keeps_the_pose_and_battery() {
        let drive: DriveShared = Default::default();
        let imu = Icm20948Input::default();
        let bat = Arc::new(Mutex::new(Ina219Input { bus_v: 11.8, current_a: 0.25, shunt_ohm: 0.01 }));
        apply(r#"{"speed":[0.3,0.3],"q":[0,0,0,1],"w":[0,0,1],"a":[0.5,0,0],"mag":[1,2,3],"bat":[11.2,3]}"#, &drive, &imu, &bat);
        disconnected(&drive, &imu);
        let p = imu.get();
        assert!(!drive.lock().unwrap().external);
        assert_eq!((p.angular_rate, p.linear_accel), ([0.0; 3], [0.0; 3]));
        assert_eq!((p.orientation, p.mag_field), ([0.0, 0.0, 0.0, 1.0], [1.0, 2.0, 3.0]));
        let b = *bat.lock().unwrap();
        assert_eq!((b.bus_v, b.current_a), (11.2, 3.0));
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
    fn lines_go_out_on_emulated_10_ms_boundaries_stamped_in_ns() {
        let (tx, rx) = std::sync::mpsc::sync_channel(QUEUE);
        let hz = 240_000_000;
        let mut link = Link::new(tx, hz);
        let d = Drive::default();
        let t = |rx: &std::sync::mpsc::Receiver<String>| rx.try_iter().map(|l| parse_json(&l).unwrap().get("t").and_then(Json::as_i64).unwrap()).collect::<Vec<_>>();
        link.send_due(0, &d, [0.0; 2]);
        assert_eq!(link.next(), hz / 100);
        link.send_due(hz / 100 - 1, &d, [0.0; 2]);
        assert_eq!(t(&rx), [0], "one line at 0, none before 10 ms");
        link.send_due(hz / 100 + 7, &d, [0.0; 2]);
        // woken late, past several boundaries: one line, at the latest, and the next on the grid
        link.send_due(hz / 100 * 5 + 3, &d, [0.0; 2]);
        assert_eq!(t(&rx), [10_000_000, 50_000_000]);
        assert_eq!(link.next(), hz / 100 * 6);
        // nobody draining: the queue fills and further lines are dropped, nothing blocks
        for k in 6..6 + 2 * QUEUE as u64 { link.send_due(hz / 100 * k, &d, [0.0; 2]); }
        assert_eq!(rx.try_iter().count(), QUEUE);
        // a guest time far beyond any run does not overflow
        link.send_due(u64::MAX, &d, [0.0; 2]);
    }

    #[test]
    fn the_board_line_carries_wheels_gimbal_and_lights() {
        let d = super::super::ros_driver::Drive { lights: [0.5, 1.0], ..Default::default() };
        let l = line(1_230_000_000, &d, [44.923, -19.956]);
        assert!(l.starts_with("{\"t\":1230000000,") && l.ends_with("}\n"));
        let j = parse_json(&l).unwrap();
        assert_eq!(numbers(&j, "led", 2), Some(vec![0.5, 1.0]));
        assert_eq!(numbers(&j, "pt", 2), Some(vec![44.92, -19.96]));
        assert_eq!(numbers(&j, "travel", 2), Some(vec![0.0, 0.0]));
    }
}
