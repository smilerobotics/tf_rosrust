use chrono::TimeDelta;
use roslibrust_codegen::Time;

pub fn stamp_now() -> roslibrust_codegen::Time {
    use std::time::SystemTime;
    let duration_since_epoch = SystemTime::now().duration_since(SystemTime::UNIX_EPOCH).unwrap();
    let stamp = roslibrust_codegen::Time {
        secs: duration_since_epoch.as_secs() as u32,
        nsecs: (duration_since_epoch.as_nanos() % 1e9 as u128) as u32,
    };
    stamp
}

pub fn stamp_to_duration(stamp: Time) -> TimeDelta
{
    TimeDelta::new(stamp.secs.into(), stamp.nsecs).unwrap()
}

pub fn duration_to_f64(time: TimeDelta) -> f64 {
    time.num_seconds() as f64 + (time.num_milliseconds() as f64 / 1000.0)
}

pub fn stamp_to_f64(stamp: Time) -> f64 {
    stamp.secs as f64 + (stamp.nsecs as f64) / 1e9
}
