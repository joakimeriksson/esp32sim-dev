//! Boards wired for the classic ESP32, independent of S3 pin assignments.
//!
//! - `TtgoTDisplay` — the LilyGO TTGO T-Display: an ST7789 135×240 on VSPI, two buttons
//! - `UlanziTc001` — the Ulanzi TC001 pixel clock: a 32×8 WS2812 matrix on RMT, three buttons
//! - `RosDriver` — Waveshare's ROS Driver for Robots (UGV Rover): motors, encoders, OLED, INA219, pan-tilt servos
//! - `NoBoard` — a bare module (`none`, `bare`, `esp32dev`): nothing on the pins, console only
//! - `esp32dev-loopback` — the SPI loopback fixture in `crate::spi`
pub mod tdisplay;
pub mod aithinker;
pub mod ulanzi;
pub mod cyd;
pub mod ros_driver;
pub mod rover_sim;

pub use tdisplay::*;

/// The physics simulation's address in `waveshare-ros-driver,sim=HOST:PORT`: `None` for a board
/// name without options, or what is wrong with them.
pub fn sim_addr(name: &str) -> Result<Option<&str>, String> {
    let Some((base, opts)) = name.split_once(',') else { return Ok(None) };
    if !matches!(base, "waveshare-ros-driver" | "ros-driver" | "ugv-rover") { return Err(format!("'{base}' takes no options; only waveshare-ros-driver takes sim=HOST:PORT")); }
    let Some(addr) = opts.strip_prefix("sim=") else { return Err(format!("unknown option '{opts}'; waveshare-ros-driver takes sim=HOST:PORT")) };
    match addr.rsplit_once(':') {
        Some((host, port)) if !host.is_empty() && port.parse::<u16>().is_ok_and(|p| p != 0) => Ok(Some(addr)),
        _ => Err(format!("sim={addr}: expected HOST:PORT with a port from 1 to 65535, e.g. sim=127.0.0.1:47000")),
    }
}

pub fn make_board(name: &str) -> Option<esp_soc::Board> {
    // `waveshare-ros-driver,sim=HOST:PORT`: the rover board linked to a physics simulation
    if let Some(addr) = sim_addr(name).ok()? {
        #[cfg(not(target_arch = "wasm32"))]
        return Some(Box::new(ros_driver::RosDriver::with_sim(addr)));
        #[cfg(target_arch = "wasm32")]
        { let _ = addr; return None; }
    }
    match name {
        "none" | "bare" | "esp32dev" => Some(Box::new(esp_soc::NoBoard)),
        "ttgo-tdisplay" | "tdisplay" => Some(Box::new(TtgoTDisplay::new())),
        "ai-thinker-cam" | "esp32cam" => Some(Box::new(aithinker::AiThinkerCam::default())),
        "cyd" | "esp32-2432s028" => Some(Box::new(cyd::Cyd::new())),
        "ulanzi-tc001" | "tc001" => Some(Box::new(ulanzi::UlanziTc001::new())),
        "waveshare-ros-driver" | "ros-driver" | "ugv-rover" => Some(Box::new(ros_driver::RosDriver::new())),
        _ => crate::spi::make_board(name),
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn classic_boards_reject_s3_names() {
        for name in ["none", "bare", "esp32dev", "ttgo-tdisplay", "tdisplay", "ulanzi-tc001", "tc001", "waveshare-ros-driver"] { assert!(super::make_board(name).is_some()); }
        for name in ["waveshare-amoled18-v2", "waveshare-lcd4b", "atech14", "waveshare-c6-lcd147"] { assert!(super::make_board(name).is_none()); }
    }

    #[test]
    fn sim_options_are_checked() {
        use super::sim_addr;
        assert_eq!(sim_addr("waveshare-ros-driver"), Ok(None));
        assert_eq!(sim_addr("ugv-rover,sim=127.0.0.1:47000"), Ok(Some("127.0.0.1:47000")));
        assert_eq!(sim_addr("ros-driver,sim=[::1]:5"), Ok(Some("[::1]:5")));
        for bad in ["waveshare-ros-driver,sim=host-without-port", "waveshare-ros-driver,sim=h:0", "waveshare-ros-driver,sim=h:99999", "waveshare-ros-driver,sim=:5", "waveshare-ros-driver,sim=h:5,x"] {
            assert!(sim_addr(bad).unwrap_err().contains("expected HOST:PORT"), "{bad}");
            assert!(super::make_board(bad).is_none());
        }
        assert!(sim_addr("waveshare-ros-driver,foo=1").unwrap_err().contains("unknown option 'foo=1'"));
        assert!(sim_addr("cyd,sim=h:5").unwrap_err().contains("'cyd' takes no options"));
    }
}
