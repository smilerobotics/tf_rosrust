use tokio::io::{AsyncBufReadExt, BufReader};
use tokio::process::Command;
use tokio::signal;

use std::process::Stdio;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut cmd0 = Command::new("rostopic");
    cmd0.arg("pub")
        .arg("/test")
        .arg("std_msgs/String")
        .arg("\"data: 'test0'\"")
        .arg("-r")
        .arg("1");

    let handle0 = tokio::spawn(async move {
        let _ = run(cmd0).await;
    });

    let mut cmd1 = Command::new("rostopic");
    cmd1.arg("echo").arg("/test");

    let handle1 = tokio::spawn(async move {
        let _ = run(cmd1).await;
    });

    match signal::ctrl_c().await {
        Ok(()) => {
            println!("got ctrl-c, exiting");
        }
        Err(err) => {
            eprintln!("Unable to listen for shutdown signal: {}", err);
            // we also shut down in case of error
        }
    }

    let rv = tokio::join!(handle0, handle1);
    println!("exited: {rv:?}");

    Ok(())
}

async fn run(mut cmd: Command) -> Result<(), Box<dyn std::error::Error>> {
    // Specify that we want the command's standard output piped back to us.
    // By default, standard input/output/error will be inherited from the
    // current process (for example, this means that standard input will
    // come from the keyboard and standard output/error will go directly to
    // the terminal if this process is invoked from the command line).
    cmd.stdout(Stdio::piped());

    let mut child = cmd.spawn().expect("failed to spawn command");

    let stdout = child
        .stdout
        .take()
        .expect("child did not have a handle to stdout");

    let mut reader = BufReader::new(stdout).lines();

    // Ensure the child process is spawned in the runtime so it can
    // make progress on its own while we await for any output.
    tokio::spawn(async move {
        let status = child
            .wait()
            .await
            .expect("child process encountered an error");

        println!("child status was: {}", status);
    });

    // TODO(lucasw) could use an mpsc channel to send this back to main
    while let Some(line) = reader.next_line().await? {
        println!("Line: {}", line);
    }

    Ok(())
}
