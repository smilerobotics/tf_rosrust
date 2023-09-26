#[derive(Debug, Clone, Copy)]
pub struct Duration {
    pub sec: i32,
    pub nanosec: u32,
}

#[cfg(feature = "ros")]
impl From<Duration> for rosrust::Duration {
    fn from(value: Duration) -> Self {
        Self {
            sec: value.sec,
            nsec: value.nanosec as i32,
        }
    }
}

#[cfg(feature = "ros")]
impl From<rosrust::Duration> for Duration {
    fn from(value: rosrust::Duration) -> Self {
        Self {
            sec: value.sec,
            nanosec: value.nsec as u32,
        }
    }
}

#[cfg(feature = "ros2")]
impl From<Duration> for r2r::builtin_interfaces::msg::Duration {
    fn from(value: Duration) -> Self {
        Self {
            sec: value.sec,
            nanosec: value.nanosec,
        }
    }
}

#[cfg(feature = "ros2")]
impl From<r2r::builtin_interfaces::msg::Duration> for Duration {
    fn from(value: r2r::builtin_interfaces::msg::Duration) -> Self {
        Self {
            sec: value.sec,
            nanosec: value.nanosec,
        }
    }
}
