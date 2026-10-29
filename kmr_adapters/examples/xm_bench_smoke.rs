//! Bench smoke test of the Dynamixel adapter, before the engine loop is
//! wired: the XmBench preset on one U2D2.
//!
//! ```sh
//! cargo run -p kmr_adapters --features dynamixel --example xm_bench_smoke -- <U2D2 serial or device path> [--torque]
//! ```
//!
//! Runs `bind`, `check`, `open`, `ping`, `health`, `configure` and
//! `read_all`, then `torque_off` and `close`. `--torque` adds `torque_on` and
//! 2 s of reads before them. No goal is ever written.

use core::fmt::Display;
use core::ops::{Deref, DerefMut};
use core::sync::atomic::{AtomicBool, Ordering};
use core::time::Duration;
use std::process::ExitCode;
use std::thread;

use kmr_adapters::binding::{Binding, Port};
use kmr_adapters::dynamixel::{DxlRs, Dynamixel, DynamixelError};
use kmr_adapters::presets::XmBench;
use kmr_core::hal::{Hal, Overrides, Sample};
use kmr_core::state::{JOINTS, Q, Qd, Tau};

/// The adapter on the `dxl_rs` SDK for the one bus of [`XmBench`].
type Hardware = Dynamixel<DxlRs, 1>;

/// Time between two reads while the torque is on.
const PERIOD: Duration = Duration::from_millis(100);

/// Reads while the torque is on: 2 s.
const READS: u32 = 20;

/// Raised by Ctrl-C; the run stops after the step in progress.
static STOP: AtomicBool = AtomicBool::new(false);

/// Why the run ended before its last step.
enum Early {
    /// A step failed.
    Failed,
    /// Ctrl-C.
    Interrupted,
}

/// The adapter once `open` was called: dropping it, on any exit or panic,
/// cuts the torque and closes the port.
struct Opened(Hardware);

impl Deref for Opened {
    type Target = Hardware;

    fn deref(&self) -> &Hardware {
        &self.0
    }
}

impl DerefMut for Opened {
    fn deref_mut(&mut self) -> &mut Hardware {
        &mut self.0
    }
}

impl Drop for Opened {
    fn drop(&mut self) {
        self.0.torque_off();
        println!("torque_off: done (an actuator that failed is logged as a warning)");
        self.0.close();
        println!("close: done");
        println!("Check Torque_Enable = 0 on all 4 servos in Dynamixel Wizard.");
    }
}

fn main() -> ExitCode {
    tracing_subscriber::fmt()
        .with_writer(std::io::stderr)
        .init();
    let Some((port, torque)) = parse() else {
        eprintln!("usage: xm_bench_smoke <U2D2 serial or device path> [--torque]");
        return ExitCode::from(2);
    };
    let handler = ctrlc::set_handler(|| {
        STOP.store(true, Ordering::SeqCst);
        eprintln!("Ctrl-C: stopping after this step");
    });
    if let Err(error) = handler {
        eprintln!("Ctrl-C handler not installed, nothing opened: {error}");
        return ExitCode::FAILURE;
    }
    match run(port, torque) {
        Ok(()) => ExitCode::SUCCESS,
        Err(Early::Failed) => ExitCode::FAILURE,
        Err(Early::Interrupted) => ExitCode::from(130),
    }
}

/// The port and whether `--torque` was given; `None` for a bad command line.
/// An argument with a `/` is a device path, any other a serial id.
fn parse() -> Option<(Port, bool)> {
    let mut port = None;
    let mut torque = false;
    for arg in std::env::args().skip(1) {
        if arg == "--torque" {
            torque = true;
        } else if arg.is_empty() || arg.starts_with('-') || port.is_some() {
            return None;
        } else if arg.contains('/') {
            port = Some(Port::ByDev(arg.leak()));
        } else {
            port = Some(Port::ById(arg.leak()));
        }
    }
    port.map(|port| (port, torque))
}

/// The steps in `Hal` order. Nothing is powered without `torque`.
fn run(port: Port, torque: bool) -> Result<(), Early> {
    let plugin = Binding {
        port,
        baud: None,
        bus: None,
    };
    let mut hardware = step(
        "bind",
        Hardware::bind(DxlRs::new(), XmBench::HARDWARE, &[plugin]),
    )?;
    let layout = step("check", hardware.check(Overrides::default()))?;
    println!(
        "  wire time: {:?} per full read, {:?} per full write",
        layout.read_cost, layout.write_cost
    );
    let mut hal = Opened(hardware);
    step("open", hal.open())?;
    step("ping", hal.ping())?;
    step("health", hal.health())?;
    step("configure", hal.configure())?;
    let mut q = [Q::default(); JOINTS];
    let (mut qd, mut tau) = ([Qd::default(); JOINTS], [Tau::default(); JOINTS]);
    step(
        "read_all",
        hal.read_all(Sample::new(&mut q, &mut qd, &mut tau)),
    )?;
    print_positions(&q);
    if !torque {
        println!("no --torque: the torque stays off");
        return Ok(());
    }
    println!(
        "torque_on: each servo holds its Goal Position register, which this program never writes"
    );
    step("torque_on", hal.torque_on())?;
    for read in 1..=READS {
        thread::sleep(PERIOD);
        let result = hal.read_all(Sample::new(&mut q, &mut qd, &mut tau));
        step(format_args!("read_all {:?}", PERIOD * read), result)?;
        print_positions(&q);
    }
    Ok(())
}

/// Prints the outcome of step `name`. Ends the run on a failure or on a
/// Ctrl-C raised meanwhile.
fn step<T>(name: impl Display, result: Result<T, DynamixelError>) -> Result<T, Early> {
    let value = match result {
        Ok(value) => value,
        Err(error) => {
            println!("{name}: FAILED: {error}");
            return Err(Early::Failed);
        }
    };
    println!("{name}: ok");
    if STOP.load(Ordering::SeqCst) {
        return Err(Early::Interrupted);
    }
    Ok(value)
}

/// Prints the position of every actuator on one line, in rad and degrees.
fn print_positions(q: &[Q; JOINTS]) {
    print!(" ");
    for (actuator, q) in XmBench::HARDWARE.actuators.iter().zip(q) {
        print!(
            " id {}: {:+.4} rad {:+8.2} deg ",
            actuator.id,
            q.0,
            q.0.to_degrees()
        );
    }
    println!();
}
