use r2r::builtin_interfaces::msg::{Duration, Time};

const BILLION: i64 = 1_000_000_000;

pub fn time_from_nanosec(t: i64) -> Time {
    Time {
        sec: (t / BILLION) as i32,
        nanosec: (t % BILLION) as u32,
    }
}

/// target - delta
pub fn sub_time_and_time(target: &Time, delta: &Time) -> Duration {
    if delta.sec.is_positive() {
        if target.nanosec >= delta.nanosec {
            Duration {
                sec: target.sec - delta.sec,
                nanosec: target.nanosec - delta.nanosec,
            }
        } else {
            Duration {
                sec: target.sec - delta.sec - 1,
                nanosec: BILLION as u32 + target.nanosec as u32 - delta.nanosec as u32,
            }
        }
    } else {
        let nanosec = target.nanosec + delta.nanosec;
        let sec = target.sec - delta.sec + nanosec as i32 / BILLION as i32;
        let nanosec = nanosec % BILLION as u32;
        Duration { sec, nanosec }
    }
}

pub fn add_time_and_duration(t: &Time, d: &Duration) -> Time {
    let sec = t.sec + d.sec;
    let nanosec = t.nanosec + d.nanosec;

    Time {
        sec: sec + nanosec as i32 / BILLION as i32,
        nanosec: nanosec % BILLION as u32,
    }
}

pub fn sub_duration_from_time(t: &Time, d: &Duration) -> Time {
    let mut sec = t.sec - d.sec;
    let mut nanosec = t.nanosec as i64 - d.nanosec as i64;

    if nanosec.is_negative() {
        sec -= 1;
        nanosec = BILLION as i64 + nanosec;
    }

    Time {
        sec,
        nanosec: nanosec as u32,
    }
}

pub fn time_as_ns_i64(t: &Time) -> i64 {
    t.sec as i64 * BILLION + t.nanosec as i64
}

pub fn is_time_in_range_eq(target: &Time, min: &Time, max: &Time) -> bool {
    let target_i64 = target.sec as i64 * BILLION + target.nanosec as i64;
    let min_i64 = min.sec as i64 * BILLION + min.nanosec as i64;
    let max_i64 = max.sec as i64 * BILLION + max.nanosec as i64;

    target_i64 >= min_i64 && target_i64 <= max_i64
}

pub fn is_time_later(target: &Time, past: &Time) -> bool {
    let target_i64 = target.sec as i64 * BILLION + target.nanosec as i64;
    let past_i64 = past.sec as i64 * BILLION + past.nanosec as i64;

    target_i64 > past_i64
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_time_from_nanosec() {
        const EXPECTED_TIME: Time = Time {
            sec: 1,
            nanosec: 234_567_890,
        };
        let nanosec = 1_234_567_890;
        let time = time_from_nanosec(nanosec);

        assert_eq!(time.sec, EXPECTED_TIME.sec);
        assert_eq!(time.nanosec, EXPECTED_TIME.nanosec);
    }

    #[test]
    fn test_sub_time_and_time() {
        const EXPECTED_DIF1: Time = Time {
            sec: 0,
            nanosec: 111_111_011,
        };
        const EXPECTED_DIF2: Time = Time {
            sec: 0,
            nanosec: 888_889_889,
        };
        const EXPECTED_DIF3: Time = Time {
            sec: 20,
            nanosec: 691_357_802,
        };

        let (t1, t2, t3, t4, _, _) = times_and_durations_for_test();

        let dif1 = sub_time_and_time(&t2, &t1);
        let dif2 = sub_time_and_time(&t2, &t3);
        let dif3 = sub_time_and_time(&t2, &t4);

        assert_eq!(dif1.sec, EXPECTED_DIF1.sec);
        assert_eq!(dif1.nanosec, EXPECTED_DIF1.nanosec);
        assert_eq!(dif2.sec, EXPECTED_DIF2.sec);
        assert_eq!(dif2.nanosec, EXPECTED_DIF2.nanosec);
        assert_eq!(dif3.sec, EXPECTED_DIF3.sec);
        assert_eq!(dif3.nanosec, EXPECTED_DIF3.nanosec);
    }

    #[test]
    fn test_add_time_and_duration() {
        const EXPECTED_SUM1: Time = Time {
            sec: 11,
            nanosec: 334_567_890,
        };
        const EXPECTED_SUM2: Time = Time {
            sec: 12,
            nanosec: 234_567_889,
        };

        let (t1, _, _, _, d1, d2) = times_and_durations_for_test();

        let sum1 = add_time_and_duration(&t1, &d1);
        let sum2 = add_time_and_duration(&t1, &d2);

        assert_eq!(sum1.sec, EXPECTED_SUM1.sec);
        assert_eq!(sum1.nanosec, EXPECTED_SUM1.nanosec);
        assert_eq!(sum2.sec, EXPECTED_SUM2.sec);
        assert_eq!(sum2.nanosec, EXPECTED_SUM2.nanosec);
    }

    #[test]
    fn test_sub_duration_from_time() {
        const EXPECTED_DIF1: Time = Time {
            sec: 9,
            nanosec: 134_567_890,
        };
        const EXPECTED_DIF2: Time = Time {
            sec: 8,
            nanosec: 234_567_891,
        };

        let (t1, _, _, _, d1, d2) = times_and_durations_for_test();

        let dif1 = sub_duration_from_time(&t1, &d1);
        let dif2 = sub_duration_from_time(&t1, &d2);

        assert_eq!(dif1.sec, EXPECTED_DIF1.sec);
        assert_eq!(dif1.nanosec, EXPECTED_DIF1.nanosec);
        assert_eq!(dif2.sec, EXPECTED_DIF2.sec);
        assert_eq!(dif2.nanosec, EXPECTED_DIF2.nanosec);
    }

    #[test]
    fn test_time_as_ns_i64() {
        const EXPECTED: i64 = 10_234_567_890;
        let (t1, _, _, _, _, _) = times_and_durations_for_test();

        let time_as_nanosec = time_as_ns_i64(&t1);

        assert_eq!(time_as_nanosec, EXPECTED);
    }

    #[test]
    fn test_is_time_in_range_eq() {
        let (t1, t2, t3, _, _, _) = times_and_durations_for_test();

        assert!(is_time_in_range_eq(&t1, &t3, &t2));
        assert!(!is_time_in_range_eq(&t2, &t3, &t1));
    }

    fn times_and_durations_for_test() -> (Time, Time, Time, Time, Duration, Duration) {
        let time1 = Time {
            sec: 10,
            nanosec: 234_567_890,
        };
        let time2 = Time {
            sec: 10,
            nanosec: 345_678_901,
        };
        let time3 = Time {
            sec: 9,
            nanosec: 456_789_012,
        };
        let time4 = Time {
            sec: -10,
            nanosec: 345_678_901,
        };
        let duration1 = Duration {
            sec: 1,
            nanosec: 100_000_000,
        };
        let duration2 = Duration {
            sec: 1,
            nanosec: 999_999_999,
        };

        (time1, time2, time3, time4, duration1, duration2)
    }
}
