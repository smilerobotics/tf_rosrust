/// Copyright 2024 Lucas WAlter
/// BSD 3-Clause
///
/// show a text version of the ros tf tree as received on /tf and /tf_static
use chrono::TimeDelta;
use clap::{arg, command};
use mcap_tools::{bins_text, get_bins, get_sorted_indices};
use roslibrust_util::tf2_msgs;
use tf_roslibrust::tf_util;
use tf_roslibrust::TfBuffer;
use tf_util::duration_to_f64;

// TODO(lucasw) have to duplicate a lot of code from mcap_tools,
// probably this should be in a separate crate
fn get_tfm_from_message(message: &mcap::Message) -> Option<tf2_msgs::TFMessage> {
    match &message.channel.schema {
        Some(schema) => {
            if schema.name == "tf2_msgs/TFMessage" {
                // println!("{}", schema.name);
                let msg_with_header =
                    roslibrust_util::get_message_data_with_header(message.data.clone());
                match serde_rosmsg::from_slice::<tf2_msgs::TFMessage>(&msg_with_header) {
                    Ok(tfm) => {
                        return Some(tfm);
                    }
                    Err(err) => {
                        log::error!("{err:?}");
                    }
                }
            }
        }
        None => {
            log::error!("no schema");
        }
    }
    None
}

fn main() -> Result<(), anyhow::Error> {
    simple_logger::SimpleLogger::new()
        .with_level(log::LevelFilter::Info)
        .init()
        .unwrap();

    let args = std::env::args();

    // TODO(lucasw) write out a toml of parent child relationships
    // like the transforms node does?  It could be a similar format,
    // static transforms would be the same but everything else could show
    // a min/max value for each xyz rpy value.
    // Also like the mcap_tools mcap_rates tool it could look for gaps in the
    // any parent-child relationship, have stats about gaps between updates
    let matches = command!()
        .arg(
            arg!(
                <mcaps> ... "mcaps to load"
            )
            .trailing_var_arg(true),
        )
        .get_matches_from(args);

    let mcap_names: Vec<_> = matches.get_many::<String>("mcaps").unwrap().collect();
    let mcap_names: Vec<String> = mcap_names.iter().map(|s| (**s).clone()).collect();
    log::info!("mcaps: {mcap_names:?}");

    let mut buffer_start_time = None;
    let mut buffer_end_time = None;
    // TODO(lucasw) make tf topics configurable
    log::info!("loading /tf and /tf_static transforms into the tf buffer");
    for mcap_name in &mcap_names {
        log::info!("getting start/end times from {mcap_name}");

        // TODO(lucasw) copied from mcap_tools which got it from the mcap rust docs, not
        // sure why it is needed.
        let mapped_mcap = roslibrust_util::map_mcap(mcap_name)?;

        let summary = mcap::read::Summary::read(&mapped_mcap)?;
        // let summary = summary.ok_or(Err(anyhow::anyhow!("no summary")))?;
        let summary = summary.ok_or(anyhow::anyhow!("{mcap_name} no summary"))?;

        // TODO(lucasw) if /tf or /tf_static aren't in this mcap, skip it

        let stats = summary
            .stats
            .ok_or(anyhow::anyhow!("{mcap_name} no stats"))?;

        let message_start_time = stats.message_start_time as f64 / 1e9;
        // TODO(lucasw) if let is better than this?
        if buffer_start_time.is_none() {
            buffer_start_time = Some(message_start_time);
        }
        buffer_start_time = Some(buffer_start_time.unwrap().min(message_start_time));

        let message_end_time = stats.message_end_time as f64 / 1e9;
        if buffer_end_time.is_none() {
            buffer_end_time = Some(message_end_time);
        }
        buffer_end_time = Some(buffer_end_time.unwrap().max(message_end_time));
    }

    let buffer_start_time = buffer_start_time.unwrap();
    let buffer_end_time = buffer_end_time.unwrap();
    let elapsed = buffer_end_time - buffer_start_time;
    log::info!("{buffer_start_time:.1} {buffer_end_time:.1} {elapsed:.1}s");
    let mut tf_buffer =
        TfBuffer::new_with_duration(chrono::TimeDelta::new(elapsed as i64, 0).unwrap());

    let tf_topics = vec!["/tf".to_string(), "/tf_static".to_string()];

    for mcap_name in mcap_names {
        log::info!("getting /tf and /tf_static from {mcap_name}");

        let mapped_mcap = roslibrust_util::map_mcap(&mcap_name)?;

        let mut count = 0;
        for message in (mcap::MessageStream::new(&mapped_mcap)?).flatten() {
            let topic = &message.channel.topic;
            if !tf_topics.contains(topic) {
                continue;
            }
            let tfm = get_tfm_from_message(&message);
            if let Some(tfm) = tfm {
                if topic == "/tf" {
                    let _ = tf_buffer.handle_incoming_transforms(tfm, false);
                } else if topic == "/tf_static" {
                    let _ = tf_buffer.handle_incoming_transforms(tfm, true);
                }
                count += 1;
            }
        }
        log::info!("{count} tfms loaded");
    }

    fn get_gap_bins(gaps: &Vec<TimeDelta>, sorted_indices: &Vec<usize>) -> (Vec<f64>, f64, f64) {
        let num = gaps.len();
        let mut gap_dvec = nalgebra::DVector::zeros(num);
        let mut gap_vec = Vec::with_capacity(num);
        for index in sorted_indices {
            let gap = duration_to_f64(gaps[*index]);
            gap_vec.push(gap);
            gap_dvec[*index] = gap;
        }

        // it's already sorted but need the indices for get_bins
        let sort_indices = get_sorted_indices(&gap_vec);
        let bins = get_bins(&gap_vec, &sort_indices, 7);
        (bins, gap_dvec.mean(), gap_dvec.variance().sqrt())
    }

    // TODO(lucasw) make this optional
    // look at each parent-child transform and look for gaps or other irregularities in the stamp of each
    let gap_datas = tf_buffer.get_gaps();
    println!("gaps in seconds between tf timestamps per-pair (with >= 2 transforms):");
    for (tf_graph_node, gap_data) in gap_datas.into_iter() {
        if let Some((_times, gaps, sort_indices)) = gap_data {
            // TODO(lucasw) sort by std_dev
            let (bins, mean, std_dev) = get_gap_bins(&gaps, &sort_indices);

            let shortest_gap = duration_to_f64(gaps[*sort_indices.first().unwrap()]);
            let longest_gap = duration_to_f64(gaps[*sort_indices.last().unwrap()]);

            // TODO(lucasw) instead of printing, put all the stats into a struct for outputting
            // into a toml file
            print!("  {:>8}    mean: {mean:.3}, std dev {std_dev:0.3}, min {shortest_gap:0.3}, max {longest_gap:0.3},", gaps.len());
            print!("    bins: {}", bins_text(&bins));
            println!(",  {} -> {}", tf_graph_node.parent, tf_graph_node.child);
        }
    }

    let _ = tf_util::print_tree(&tf_buffer);

    Ok(())
}
