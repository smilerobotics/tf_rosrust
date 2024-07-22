pub fn stamp_now() -> roslibrust_codegen::Time {
    use std::time::SystemTime;
    let duration_since_epoch = SystemTime::now().duration_since(SystemTime::UNIX_EPOCH).unwrap();
    let stamp = roslibrust_codegen::Time {
        secs: duration_since_epoch.as_secs() as u32,
        nsecs: (duration_since_epoch.as_nanos() % 1e9 as u128) as u32,
    };
    stamp
}
