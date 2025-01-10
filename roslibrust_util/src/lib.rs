use anyhow::{Context, Result};
use camino::Utf8Path;
use memmap::Mmap;
use roslibrust::ros1::{self, determine_addr, MasterClient, NodeServerHandle, XmlRpcServer};
use roslibrust::RosMessageType;
use serde_derive::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use tokio::sync::mpsc;

roslibrust::find_and_generate_ros_messages!();

pub async fn get_master_client(node_name: &str) -> Result<MasterClient, anyhow::Error> {
    // copied this out of roslibrust actor.rs Node::new(), seemed like bare minimum
    // to make a valid master client
    let master_uri =
        std::env::var("ROS_MASTER_URI").unwrap_or("http://localhost:11311".to_string());

    let (node_sender, _node_receiver) = mpsc::unbounded_channel();
    let xml_server_handle = NodeServerHandle {
        node_server_sender: node_sender.clone(),
        // None here because this handle should not keep task alive
        _node_task: None,
    };

    let (addr, hostname) = determine_addr().await?;

    // Create our xmlrpc server and bind our socket so we know our port and can determine our local URI
    let xmlrpc_server = XmlRpcServer::new(addr, xml_server_handle)?;
    let client_uri = format!("http://{hostname}:{}", xmlrpc_server.port());

    let master_client = MasterClient::new(
        master_uri.clone(),
        client_uri.clone(),
        node_name.to_string(),
    )
    .await?;

    tracing::info!("{node_name} connected to roscore at {master_uri} from {client_uri}");

    Ok(master_client)
}

/// pass in empty hashmap or set it up with params to look for
/// let mut params = HashMap::<String, String>::new();
/// params.insert("update_rate".to_string(), "5.0".to_string());
///
/// returns full path node name, the namespace, and a vector of unused args
pub fn get_params_remaps(
    params: &mut HashMap<String, String>,
    remaps: &mut HashMap<String, String>,
) -> (String, String, Vec<String>) {
    // TODO(lucasw) generate a unique node name
    // let _ = params.try_insert("_name".to_string(), "node_tbd".to_string());
    if !params.contains_key("_name") {
        params.insert("_name".to_string(), "node_tbd".to_string());
    }
    if !params.contains_key("_log") {
        // TODO(lucasw) generate a random string
        // TODO(lucasw) not actually doing anything with these logs
        params.insert("_log".to_string(), "tbd_node_logs.log".to_string());
    }
    params.insert("_ns".to_string(), "".to_string());

    // TODO(lucasw) can an existing rust arg handling library handle the ':=' ros cli args?
    let args = std::env::args();
    let mut args2 = Vec::new();
    for arg in args {
        let key_val: Vec<&str> = arg.split(":=").collect();
        if key_val.len() != 2 {
            args2.push(arg);
            continue;
        }

        let (mut key, val) = (key_val[0].to_string(), key_val[1].to_string());
        if !key.starts_with('_') {
            remaps.insert(key, val);
            continue;
        }
        key.replace_range(0..1, "");

        if !params.contains_key(&key) {
            tracing::warn!("unexpected param: '{key}' '{val}'");
            // continue;
        }
        params.insert(key, val);
    }
    tracing::info!("{args2:?}");

    let ns = params.remove("_ns").unwrap();
    let full_node_name = &format!("/{}/{}", &ns, &params["_name"],).replace("//", "/");

    let mut updated_remaps = HashMap::new();
    // if the topic is relative, add the namespace here and update the remaps
    for (topic_orig, topic_remap) in &mut *remaps {
        let topic_orig = topic_orig.clone();
        if !topic_remap.starts_with('/') {
            let updated_topic_remap = format!("{ns}/{}", topic_remap).to_string();
            updated_remaps.insert(topic_orig, updated_topic_remap);
        } else {
            updated_remaps.insert(topic_orig, topic_remap.to_string());
        }
        // TODO(lucasw) handle private topics with '~' leading?
    }
    *remaps = updated_remaps;
    /*
    for (topic_orig, topic_remap) in updated_remaps {
        remaps.insert(topic_orig, topic_remap);
    }
    */

    (ns.to_string(), full_node_name.to_string(), args2)
}

// TODO(lucasw) why is this needed?  https://docs.rs/mcap/latest/mcap/ doesn't explain it
pub fn map_mcap<P: AsRef<Utf8Path>>(p: P) -> Result<Mmap> {
    let fd = std::fs::File::open(p.as_ref()).context("Couldn't open MCAP file")?;
    unsafe { Mmap::map(&fd) }.context("Couldn't map MCAP file")
}

// TODO(lucasw) https://github.com/Carter12s/roslibrust/issues/158#issuecomment-2187839437
pub fn get_message_data_with_header(raw_message_data: std::borrow::Cow<'_, [u8]>) -> Vec<u8> {
    let len_header = raw_message_data.len() as u32;
    let mut msg_with_header = Vec::from(len_header.to_le_bytes());
    let mut message_data = Vec::from(raw_message_data);
    msg_with_header.append(&mut message_data);
    msg_with_header
}

// for reading/writing tomls
#[derive(Deserialize, Serialize, Debug)]
pub struct TopicStats {
    pub topic: String,
    pub topic_type: String,
    pub rate: Option<f64>,
}

// TODO(lucasw) maybe should just use anyhow::Error, can't use the question mark with this
#[derive(Debug, thiserror::Error)]
pub enum ReadTomlFileError {
    #[error("couldn't read file")]
    Read(std::io::Error),
    #[error("couldn't parse toml")]
    Toml(toml::de::Error),
}

pub fn get_expected_rates_from_toml(
    input_toml_name: &String,
) -> Result<HashMap<String, TopicStats>, ReadTomlFileError> {
    // let contents = std::fs::read_to_string(input_toml_name)?;
    let contents = match std::fs::read_to_string(input_toml_name) {
        Ok(contents) => contents,
        Err(err) => {
            return Err(ReadTomlFileError::Read(err));
            // panic!("Could not read file '{input_toml_name}', {err}");
        }
    };
    let mut rates_data: HashMap<String, Vec<TopicStats>> = match toml::from_str(&contents) {
        Ok(rates_data) => rates_data,
        Err(err) => {
            return Err(ReadTomlFileError::Toml(err));
        }
    };
    let mut rates = HashMap::new();
    let rates_vec = rates_data.remove("topic_stats").unwrap();
    for rate in rates_vec {
        rates.insert(rate.topic.clone(), rate);
    }
    Ok(rates)
}

// TODO(lucasw) this wouldn't be needed if Subscriber had try_recv
fn sub_to_arc_mutex<T: RosMessageType>(
    name: &str,
    mut sub: ros1::Subscriber<T>,
    value: Arc<Mutex<Option<T>>>,
) -> tokio::task::JoinHandle<()> {
    let name = name.to_string();
    tokio::spawn(async move {
        loop {
            tokio::select! {
                _ = tokio::signal::ctrl_c() => {
                    log::warn!("[{name}] subscriber ctrl-c, shutting down subscriber-to-local-channel");
                    break;
                }
                // This next will wait forever if nothing is received so need the select to break out and
                // exit- but maybe dropping the node handle can be made to unregister subscriber
                rv = sub.next() => {
                    match rv {
                        Some(rv) => {
                            match rv {
                                // TODO(lucasw) this will block if the receiver isn't getting
                                // drained, which will caused the above sub.next() to fail?
                                Ok(msg) => match value.lock() {
                                    Ok(mut value) => {
                                        *value = Some(msg);
                                    }
                                    Err(err) => {
                                        log::warn!("[{name}] sub ok but can't lock value {err:?}");
                                        break;
                                    }
                                },
                                /*
                                Err(ros1::Subscriber::SubscriberError::Lagged(n)) => {
                                    log::warn!("{:?} sub lagged {n}", T::ROS_TYPE_NAME);
                                    continue;
                                }
                                */
                                Err(err) => {
                                    log::warn!("[{name}] subscriber {err:?}");
                                    break;
                                }
                            }
                        }
                        None => {
                            log::warn!("[{name}] No messages, subscriber is done {}", T::ROS_TYPE_NAME);
                            break;
                        }
                    }
                }
            }
        }
    })
}

// #[derive(Clone)] can't clone the handle, the caller just needs to clone the arc mutex
pub struct LatestFromSubscriber<T: RosMessageType> {
    _handle: tokio::task::JoinHandle<()>,
    pub latest: Arc<Mutex<Option<T>>>,
}

impl<T: RosMessageType> LatestFromSubscriber<T> {
    pub fn new(name: &str, sub: ros1::Subscriber<T>) -> Self {
        let latest = Arc::new(Mutex::new(None));
        let handle = sub_to_arc_mutex::<T>(name, sub, latest.clone());
        Self {
            _handle: handle,
            latest,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // TODO(lucasw) ignoring for now, need CI to run a roscore
    #[ignore]
    #[tokio::test]
    async fn latest_from_sub() -> Result<(), anyhow::Error> {
        // roscore needs to be running
        let mut params = HashMap::<String, String>::new();
        params.insert("_name".to_string(), "odom_latest".to_string());
        let mut remaps = HashMap::<String, String>::new();
        remaps.insert("odom".into(), "odom".into());
        let (_ns, full_node_name, _remaining_args) = get_params_remaps(&mut params, &mut remaps);

        let master_uri =
            std::env::var("ROS_MASTER_URI").unwrap_or("http://localhost:11311".to_string());

        let odom_topic = remaps.get("odom").context("no odom remap")?;

        let nh = ros1::NodeHandle::new(&master_uri, &full_node_name).await?;

        let odom_sub = nh.subscribe::<nav_msgs::Odometry>(odom_topic, 20).await?;

        let latest_odom = LatestFromSubscriber::new("odom", odom_sub);
        let _latest_odom_arc_mutex = latest_odom.latest.clone();

        Ok(())
    }
}
