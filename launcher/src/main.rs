use roslibrust_util::ReadTomlFileError;
use serde::de::Error;
use serde_derive::{Deserialize, Serialize};
use std::collections::HashMap;
use tokio::io::{AsyncBufReadExt, BufReader};
use tokio::process::Command;
use tokio::signal;

use std::process::Stdio;

#[derive(Deserialize, Serialize, Debug)]
pub struct Cmd {
    pub name: String,
    pub command: String,
    pub args: Vec<String>,
}

pub fn get_cmds_from_toml(
    input_toml_name: &String,
    // ) -> Result<HashMap<String, Cmd>, ReadTomlFileError> {
) -> Result<Vec<Cmd>, ReadTomlFileError> {
    println!("opening '{input_toml_name}'");
    let contents = match std::fs::read_to_string(input_toml_name) {
        Ok(contents) => contents,
        Err(err) => {
            return Err(ReadTomlFileError::Read(err));
        }
    };
    let mut data: HashMap<String, Vec<Cmd>> = match toml::from_str(&contents) {
        Ok(data) => data,
        Err(err) => {
            return Err(ReadTomlFileError::Toml(err));
        }
    };
    // let mut cmds = HashMap::new();
    let cmd_vec = match data.remove("cmd") {
        Some(cmd_vec) => cmd_vec,
        None => {
            // TODO(lucasw) need a third read toml error type, or just return empty vec?
            return Err(ReadTomlFileError::Toml(toml::de::Error::missing_field(
                "no cmds in toml",
            )));
        }
    };
    // TODO(lucasw) error on duplicate names... or just make the toml have a proper table of
    // structs format
    Ok(cmd_vec)
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    let input_toml_name = &args[1];
    let cmds = get_cmds_from_toml(input_toml_name)?;

    let mut handles = Vec::new();
    for cmd in cmds {
        let handle = tokio::spawn(async move {
            let _ = run(cmd).await;
        });
        handles.push(handle);
    }

    use futures::future::TryJoinAll;
    tokio::select! {
        _ = signal::ctrl_c() => {
            println!("got ctrl-c, exiting");
            // break;
        }
        rv = handles.into_iter().collect::<TryJoinAll<_>>() => {
            println!("all handles finished, exiting {rv:?}");
            // break;
        }
    }

    Ok(())
}

async fn run(cmd: Cmd) -> Result<(), Box<dyn std::error::Error>> {
    let mut command = Command::new(cmd.command);
    for arg in cmd.args {
        command.arg(arg);
    }

    // Specify that we want the command's standard output piped back to us.
    // By default, standard input/output/error will be inherited from the
    // current process (for example, this means that standard input will
    // come from the keyboard and standard output/error will go directly to
    // the terminal if this process is invoked from the command line).

    // https://stackoverflow.com/questions/49245907/how-to-read-subprocess-output-asynchronously
    // https://stackoverflow.com/a/72403240/603653
    let error_msg = format!("'{}' failed to spawn command", cmd.name);
    let mut child = command
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .kill_on_drop(true)
        .spawn()
        .expect(&error_msg);

    let error_msg = format!("'{}' child did not have a handle to stdout", cmd.name);
    let stdout = child.stdout.take().expect(&error_msg);
    let mut stdout_reader = BufReader::new(stdout).lines();

    let error_msg = format!("'{}' child did not have a handle to stderr", cmd.name);
    let stderr = child.stderr.take().expect(&error_msg);
    let mut stderr_reader = BufReader::new(stderr).lines();

    loop {
        tokio::select! {
            result = child.wait() => {
                println!("[{}] child exited {result:?}", cmd.name);
                // TODO(lucasw) return error if non-zero
                // match result {
                // }
                break // child process exited
            }
            result = stdout_reader.next_line() => {
                match result {
                    Ok(Some(line)) => { println!("[{}] {line}", cmd.name); }
                    Err(err) => {
                        eprintln!("exiting after stdout reader error: {err:?}");
                        return Err(err.into());
                    }
                    _ => (), // TODO(lucasw) what happened here?
                }
            }
            result = stderr_reader.next_line() => {
                match result {
                    Ok(Some(line)) => { println!("[{}] {line}", cmd.name); }
                    Err(err) => {
                        eprintln!("[{}] exiting after stderr reader error: {err:?}", cmd.name);
                        return Err(err.into());
                    }
                    _ => (), // TODO(lucasw) what happened here?
                }
            }
        }
    }

    println!("[{}] done", cmd.name);
    Ok(())
}
