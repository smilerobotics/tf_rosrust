use clap::command;
use roslibrust_util::ReadTomlFileError;
// use serde::de::Error;
use serde_derive::{Deserialize, Serialize};
use std::collections::HashMap;
use std::process::ExitStatus;
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

#[derive(Clone, Deserialize, Serialize, Debug)]
pub struct Arg {
    pub name: String,
    pub default: String,
}

#[derive(Deserialize, Serialize, Debug)]
pub struct Launch {
    pub arg: Vec<Arg>,
    pub cmd: Vec<Cmd>,
}

pub fn get_cmds_from_toml(input_toml_name: &String) -> Result<Launch, ReadTomlFileError> {
    println!("opening '{input_toml_name}'");
    let contents = match std::fs::read_to_string(input_toml_name) {
        Ok(contents) => contents,
        Err(err) => {
            return Err(ReadTomlFileError::Read(err));
        }
    };
    let data: Launch = match toml::from_str(&contents) {
        Ok(data) => data,
        Err(err) => {
            return Err(ReadTomlFileError::Toml(err));
        }
    };
    Ok(data)
}

fn make_launch_handles(
    cmds: Vec<Cmd>,
    launch_args: HashMap<String, String>,
) -> Vec<tokio::task::JoinHandle<ExitStatus>> {
    let mut handles = Vec::new();
    for cmd in cmds {
        print!("[{}] {}", cmd.name, cmd.command);
        let mut command = Command::new(cmd.command);
        for arg in cmd.args {
            let mut arg = arg.clone();
            for (launch_arg_name, launch_arg_value) in &launch_args {
                arg = arg.replace(&format!("$(arg {})", launch_arg_name), launch_arg_value);
            }
            print!(" {arg}");
            command.arg(arg);
        }
        println!();

        let handle = tokio::spawn(async move {
            let rv = run(command, cmd.name).await;
            match rv {
                Ok(es) => es,
                Err(err) => {
                    eprintln!("{err:?}");
                    // TODO(lucasw) not sure about this but it compiles
                    // need to map errors to unix exit codes as much as possible but for now any
                    // non-zero works
                    std::os::unix::prelude::ExitStatusExt::from_raw(1)
                }
            }
        });
        handles.push(handle);
    }
    handles
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let cli_args: Vec<String> = std::env::args().collect();
    let input_toml_name = &cli_args[1];
    let launch = get_cmds_from_toml(input_toml_name)?;
    let cmds = launch.cmd;
    let mut launch_args = HashMap::new();
    for arg in launch.arg {
        launch_args.insert(arg.name, arg.default);
    }

    let mut remaining_args = Vec::new();
    // override the toml default values with command line provided ones
    for cli_arg in &cli_args[2..] {
        let key_val: Vec<&str> = cli_arg.split(":=").collect();
        if key_val.len() != 2 {
            remaining_args.push(cli_arg);
            continue;
        }
        let key = key_val[0];
        let value = key_val[1];
        if !launch_args.contains_key(key) {
            return Err(format!("unexpected arg {key} (set to {value})").into());
        }
        println!("overriding {key} default value with {value}");
        launch_args.insert(key.to_string(), value.to_string());
    }

    // use clap just for the --version currently
    let _matches = command!().get_matches_from(remaining_args);

    let handles = make_launch_handles(cmds, launch_args);

    tokio::spawn(async move {
        signal::ctrl_c().await.unwrap();
        println!("got ctrl-c, waiting for handles to end before exiting");
        tokio::time::sleep(tokio::time::Duration::from_secs(4)).await;
        eprintln!("failed to join handles after timeout");
        std::process::exit(1);
    });

    use futures::future::TryJoinAll;
    let codes = handles.into_iter().collect::<TryJoinAll<_>>().await?;
    for code in &codes {
        // return if any non-zero
        if !code.success() {
            return Err(format!("{codes:?}").into());
        }
    }
    println!("all handles finished without error, exiting");

    Ok(())
}

async fn run(
    mut command: Command,
    cmd_name: String,
) -> Result<ExitStatus, Box<dyn std::error::Error>> {
    // Specify that we want the command's standard output piped back to us.
    // By default, standard input/output/error will be inherited from the
    // current process (for example, this means that standard input will
    // come from the keyboard and standard output/error will go directly to
    // the terminal if this process is invoked from the command line).

    // https://stackoverflow.com/questions/49245907/how-to-read-subprocess-output-asynchronously
    // https://stackoverflow.com/a/72403240/603653
    let error_msg = format!("'{}' failed to spawn command", cmd_name);
    let mut child = command
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .kill_on_drop(true)
        .spawn()
        .expect(&error_msg);

    let error_msg = format!("'{}' child did not have a handle to stdout", cmd_name);
    let stdout = child.stdout.take().expect(&error_msg);
    let mut stdout_reader = BufReader::new(stdout).lines();

    let error_msg = format!("'{}' child did not have a handle to stderr", cmd_name);
    let stderr = child.stderr.take().expect(&error_msg);
    let mut stderr_reader = BufReader::new(stderr).lines();

    loop {
        tokio::select! {
            exit_status = child.wait() => {
                println!("[{}] child exited {exit_status:?}", cmd_name);
                // TODO(lucasw) return error if non-zero
                match exit_status {
                    Ok(exit_status) => {
                        return Ok(exit_status);
                    }
                    Err(err) => {
                        return Err(err.into());
                    }
                }
            }
            result = stdout_reader.next_line() => {
                match result {
                    Ok(Some(line)) => { println!("[{}] {line}", cmd_name); }
                    Err(err) => {
                        eprintln!("exiting after stdout reader error: {err:?}");
                        return Err(err.into());
                    }
                    _ => (), // TODO(lucasw) what happened here?
                }
            }
            result = stderr_reader.next_line() => {
                match result {
                    Ok(Some(line)) => { println!("[{}] {line}", cmd_name); }
                    Err(err) => {
                        eprintln!("[{}] exiting after stderr reader error: {err:?}", cmd_name);
                        return Err(err.into());
                    }
                    _ => (), // TODO(lucasw) what happened here?
                }
            }
        }
    }

    // println!("[{}] done", cmd_name);
    // Ok(ExitStatus::default())
}
