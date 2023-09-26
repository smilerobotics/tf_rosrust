#[derive(Debug, Clone, Copy)]
pub struct Time {
    pub sec: i32,
    pub nanosec: u32,
}

#[cfg(feature = "ros")]
impl From<Time> for rosrust::Time {
    fn from(value: Time) -> Self {
        Self {
            sec: value.sec as u32,
            nsec: value.nanosec,
        }
    }
}

#[cfg(feature = "ros")]
impl From<rosrust::Time> for Time {
    fn from(value: rosrust::Time) -> Self {
        Self {
            sec: value.sec as i32,
            nanosec: value.nsec,
        }
    }
}

#[cfg(feature = "ros2")]
impl From<Time> for r2r::builtin_interfaces::msg::Time {
    fn from(value: Time) -> Self {
        Self {
            sec: value.sec,
            nanosec: value.nanosec,
        }
    }
}

#[cfg(feature = "ros2")]
impl From<r2r::builtin_interfaces::msg::Time> for Time {
    fn from(value: r2r::builtin_interfaces::msg::Time) -> Self {
        Self {
            sec: value.sec,
            nanosec: value.nanosec,
        }
    }
}
