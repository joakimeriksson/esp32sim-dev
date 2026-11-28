use std::{path::PathBuf, process::Command};

/// esp32sim on the firmware in `UGV_BASE_FW_DIR` with `script`, for `seconds`: stdout and stderr.
fn run_ugv_base(name: &str, script: &str, seconds: &str) -> String {
    let dir = PathBuf::from(std::env::var("UGV_BASE_FW_DIR")
        .expect("set UGV_BASE_FW_DIR to ugv_base_ros's .pio/build/<env> (bootloader.bin, partitions.bin, firmware.bin)"));
    let rom = esp_soc::rom::require("esp32_rev300_rom.elf");
    let path = std::env::temp_dir().join(format!("{name}-{}.txt", std::process::id()));
    std::fs::write(&path, script).unwrap();
    let out = Command::new(env!("CARGO_BIN_EXE_esp32sim"))
        .args(["--chip", "esp32", "--board", "waveshare-ros-driver", "--boot", "rom", "--rom"]).arg(&rom)
        .arg("--bootloader").arg(dir.join("bootloader.bin"))
        .arg("--ptable").arg(dir.join("partitions.bin"))
        .arg("--app").arg(dir.join("firmware.bin"))
        .arg("--script").arg(&path)
        .args(["--net", "none", "--max-seconds", seconds, "--no-dump"])
        .output().expect("run esp32sim");
    let _ = std::fs::remove_file(&path);
    String::from_utf8_lossy(&out.stdout).into_owned() + &String::from_utf8_lossy(&out.stderr)
}
fn field(line: &str, key: &str) -> Option<f64> {
    line.split(&format!("\"{key}\":")).nth(1).and_then(|v| v.split([',', '}']).next()).and_then(|v| v.parse::<f64>().ok())
}

/// esp32sim's exit status and stderr for `args`.
fn cli(args: &[&str]) -> (Option<i32>, String) {
    let out = Command::new(env!("CARGO_BIN_EXE_esp32sim")).args(args).output().expect("run esp32sim");
    (out.status.code(), String::from_utf8_lossy(&out.stderr).into_owned())
}

#[test]
fn the_board_is_listed_and_a_bad_sim_option_says_what_is_wrong() {
    let (code, help) = cli(&["--help"]);
    assert_eq!(code, Some(0));
    assert!(help.contains("waveshare-ros-driver,sim=HOST:PORT"), "{help}");
    let (code, err) = cli(&["--chip", "esp32", "--board", "unknown"]);
    assert_eq!(code, Some(2));
    assert!(err.contains("waveshare-ros-driver"), "{err}");
    let (code, err) = cli(&["--chip", "esp32", "--board", "waveshare-ros-driver,sim=host-without-port"]);
    assert_eq!(code, Some(2));
    assert!(err.contains("--board waveshare-ros-driver,sim=host-without-port: sim=host-without-port: expected HOST:PORT") && !err.contains("unknown classic"), "{err}");
    let (code, err) = cli(&["--chip", "esp32", "--board", "ttgo-tdisplay,sim=127.0.0.1:5"]);
    assert_eq!(code, Some(2));
    assert!(err.contains("'ttgo-tdisplay' takes no options"), "{err}");
}

/// Waveshare's ugv_base_ros firmware (waveshareteam/ugv_base_ros at 2e7df97, GPL-3.0, built
/// unmodified with PlatformIO: bootloader.bin, partitions.bin, firmware.bin) on the
/// `waveshare-ros-driver` board. Not redistributed here.
#[test]
#[ignore = "set UGV_BASE_FW_DIR to a ugv_base_ros PlatformIO build and ESP32SIM_ROM_DIR to esp32_rev300_rom.elf"]
fn external_ugv_base_drives_and_reports_odometry() {
    // The firmware's T:1 is a closed-loop speed command in m/s; it streams T:1001 feedback lines.
    let text = run_ugv_base("ugv-rover", "0 waituart0 30 \"T\":1001\n0 uart0 {\"T\":1,\"L\":0.3,\"R\":-0.2}\n", "9");
    assert!(text.contains("DMP enabled!"), "the ICM-20948 passes the SparkFun DMP setup:\n{text}");
    let last = text.lines().rfind(|l| l.contains("\"T\":1001")).expect("T:1001 feedback");
    let (odl, odr, v) = (field(last, "odl").unwrap(), field(last, "odr").unwrap(), field(last, "v").unwrap());
    // About 3 s of motion before the heartbeat (3 s) stops the wheels: left forward, right back.
    assert!(odl > 50.0 && odr < -35.0, "odometry in cm: {last}");
    assert!((odl / -odr - 1.5).abs() < 0.3, "the PID holds the commanded ratio: {last}");
    assert!((v - 1180.0).abs() < 50.0, "11.8 V from the INA219: {last}");
}

/// The same firmware with its pan-tilt module: T:900 sets moduleType 2 (the gimbal), T:133 points
/// it, and the two ST3215 servos on UART1 move there and report back through T:1001.
#[test]
#[ignore = "set UGV_BASE_FW_DIR to a ugv_base_ros PlatformIO build and ESP32SIM_ROM_DIR to esp32_rev300_rom.elf"]
fn external_ugv_base_points_the_gimbal() {
    let text = run_ugv_base("ugv-gimbal", "0 waituart0 30 \"T\":1001\n0 uart0 {\"T\":900,\"main\":2,\"module\":2}\n\
                                           0 uart0 {\"T\":133,\"X\":45,\"Y\":20,\"SPD\":0,\"ACC\":0}\n", "8");
    let pans: Vec<f64> = text.lines().filter(|l| l.contains("\"T\":1001")).filter_map(|l| field(l, "pan")).collect();
    assert!(pans.len() > 10, "T:1001 reports the gimbal once moduleType is 2:\n{text}");
    assert!(pans.iter().any(|&p| p > 1.0 && p < 44.0), "the pan servo travels at its speed, not at once: {pans:?}");
    let last = text.lines().rfind(|l| l.contains("\"T\":1001")).unwrap();
    let (pan, tilt) = (field(last, "pan").unwrap(), field(last, "tilt").unwrap());
    // gimbal_module.h maps 45 and 20 degrees to whole steps (Arduino map): 44.92 and 19.96
    assert!((pan - 45.0).abs() < 0.2 && (tilt - 20.0).abs() < 0.2, "settled at the command: {last}");
    assert!(text.contains("gimbal pan +44.9 tilt +20.0 deg"), "the board's report:\n{text}");
}
