//! Bench smoke test of the Dynamixel adapter, before the engine loop is
//! wired: the XmBench preset, or with `--preset rx28` the Rx28Chain one, on
//! one USB serial adapter.
//!
//! ```sh
//! cargo run -p kmr_adapters --features dynamixel --example xm_bench_smoke -- <U2D2 serial or device path> [--torque]
//! cargo run -p kmr_adapters --features dynamixel --example xm_bench_smoke -- --preset rx28 <serial or device path> [--torque]
//! ```
//!
//! Runs `bind`, `check`, `open`, `ping`, `health`, `configure` and
//! `read_all`, then `torque_off` and `close`. `--torque` adds `torque_on` and
//! 2 s of reads before them. The only goal written is `torque_on`'s: each
//! servo's present position, written before Torque Enable, so none jumps to
//! an old goal.

use core::fmt::Display;
use core::ops::{Deref, DerefMut};
use core::sync::atomic::{AtomicBool, Ordering};
use core::time::Duration;
use std::process::ExitCode;
use std::thread;

use kmr_adapters::binding::{Binding, Port};
use kmr_adapters::dynamixel::{DxlRsV2, Dynamixel, DynamixelError, Table};
use kmr_adapters::presets::{Rx28Chain, XmBench};
use kmr_core::hal::{Hal, Overrides, Sample};
use kmr_core::state::{Kd, Kp, Q, Qd, Tau};

/// The adapter on the `dxl_rs_v2` SDK for the one bus of a preset of `N`
/// actuators.
type Hardware<const N: usize> = Dynamixel<DxlRsV2<N>, 1, N>;

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
struct Opened<const N: usize>(Hardware<N>);

impl<const N: usize> Deref for Opened<N> {
    type Target = Hardware<N>;

    fn deref(&self) -> &Hardware<N> {
        &self.0
    }
}

impl<const N: usize> DerefMut for Opened<N> {
    fn deref_mut(&mut self) -> &mut Hardware<N> {
        &mut self.0
    }
}

impl<const N: usize> Drop for Opened<N> {
    fn drop(&mut self) {
        self.0.torque_off();
        println!("torque_off: done (an actuator that failed is logged as a warning)");
        self.0.close();
        println!("close: done");
        println!("Check Torque_Enable = 0 on all {N} servos in Dynamixel Wizard.");
    }
}

fn main() -> ExitCode {
    tracing_subscriber::fmt()
        .with_writer(std::io::stderr)
        .init();
    let Some((port, torque, rx28)) = parse() else {
        eprintln!(
            "usage: xm_bench_smoke [--preset xm_bench|rx28] <U2D2 serial or device path> [--torque]"
        );
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
    let result = if rx28 {
        run(Rx28Chain::HARDWARE, port, torque)
    } else {
        run(XmBench::HARDWARE, port, torque)
    };
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(Early::Failed) => ExitCode::FAILURE,
        Err(Early::Interrupted) => ExitCode::from(130),
    }
}

/// The port, whether `--torque` was given and whether `--preset rx28` was;
/// `None` for a bad command line. An argument with a `/` is a device path,
/// any other a serial id.
fn parse() -> Option<(Port, bool, bool)> {
    let mut port = None;
    let mut torque = false;
    let mut rx28 = false;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        if arg == "--torque" {
            torque = true;
        } else if arg == "--preset" {
            rx28 = match args.next()?.as_str() {
                "rx28" => true,
                "xm_bench" => false,
                _ => return None,
            };
        } else if arg.is_empty() || arg.starts_with('-') || port.is_some() {
            return None;
        } else if arg.contains('/') {
            port = Some(Port::ByDev(arg.leak()));
        } else {
            port = Some(Port::ById(arg.leak()));
        }
    }
    port.map(|port| (port, torque, rx28))
}

/// The steps in `Hal` order on `table`. Nothing is powered without `torque`.
fn run<const N: usize>(table: Table<1, N>, port: Port, torque: bool) -> Result<(), Early> {
    let plugin = Binding {
        port,
        baud: None,
        bus: None,
    };
    let mut hardware = step(
        "bind",
        Hardware::<N>::bind(DxlRsV2::new(), table.in_order(), &[plugin]),
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
    let mut q = [Q::default(); N];
    let (mut qd, mut tau) = ([Qd::default(); N], [Tau::default(); N]);
    let (mut kp, mut kd) = ([Kp::default(); N], [Kd::default(); N]);
    step(
        "read_all",
        hal.read_all(Sample::new(&mut q, &mut qd, &mut tau, &mut kp, &mut kd)),
    )?;
    print_positions(&table, &q);
    if !torque {
        println!("no --torque: the torque stays off");
        return Ok(());
    }
    println!(
        "torque_on: Goal Position = present position, written with the torque off, then torque on"
    );
    step("torque_on", hal.torque_on())?;
    for read in 1..=READS {
        thread::sleep(PERIOD);
        let result = hal.read_all(Sample::new(&mut q, &mut qd, &mut tau, &mut kp, &mut kd));
        step(format_args!("read_all {:?}", PERIOD * read), result)?;
        print_positions(&table, &q);
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

/// Prints the position of every actuator of `table` on one line, in rad and
/// degrees.
fn print_positions<const N: usize>(table: &Table<1, N>, q: &[Q; N]) {
    print!(" ");
    for (actuator, q) in table.actuators.iter().zip(q) {
        print!(
            " id {}: {:+.4} rad {:+8.2} deg ",
            actuator.id,
            q.0,
            q.0.to_degrees()
        );
    }
    println!();
}
