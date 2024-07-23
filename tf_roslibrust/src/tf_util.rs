use chrono::TimeDelta;
use roslibrust_codegen::Time;
use std::time::SystemTime;

pub fn to_stamp(secs: u32, nsecs: u32) -> Time {
    roslibrust_codegen::Time {
        secs,
        nsecs,
    }
}

pub fn duration_now() -> TimeDelta {
    let elapsed = SystemTime::now().duration_since(SystemTime::UNIX_EPOCH).unwrap();
    // println!("{} {}", elapsed.as_secs(), elapsed.subsec_nanos());
    TimeDelta::new(elapsed.as_secs() as i64, elapsed.subsec_nanos() as u32).unwrap()
}

pub fn stamp_now() -> roslibrust_codegen::Time {
    let duration_since_epoch = duration_now();
    to_stamp(
        duration_since_epoch.num_seconds() as u32,
        duration_since_epoch.subsec_nanos() as u32,
    )
}

pub fn stamp_to_duration(stamp: Time) -> TimeDelta
{
    TimeDelta::new(stamp.secs.into(), stamp.nsecs).unwrap()
}

pub fn duration_to_f64(time: TimeDelta) -> f64 {
    time.num_seconds() as f64 + (time.subsec_nanos() as f64 / 1e9)
}

pub fn stamp_to_f64(stamp: Time) -> f64 {
    stamp.secs as f64 + (stamp.nsecs as f64) / 1e9
}
