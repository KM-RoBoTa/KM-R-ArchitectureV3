//! Under feature `pinocchio`: embeds the GeckoLeg URDF for Pinocchio, links Pinocchio through
//! pkg-config, and compiles src/geckoleg.cpp for the robot `robot!` names in src/main.rs.
//! Without the feature it does nothing.

#[cfg(feature = "pinocchio")]
fn main() {
    if let Err(e) = pinocchio::build() {
        eprintln!("{e}");
        std::process::exit(1);
    }
}

#[cfg(not(feature = "pinocchio"))]
fn main() {
    println!("cargo:rerun-if-changed=build.rs");
}

#[cfg(feature = "pinocchio")]
mod pinocchio {
    use std::path::{Path, PathBuf};
    use std::process::Command;

    /// The one model of the leg: `robot!` and Pinocchio both read it.
    const URDF: &str = "../../../controllers/models/geckoleg.urdf";

    /// The command that builds and runs the example, for the error messages.
    const RUN: &str = "CXXFLAGS=\"$(pkg-config --cflags pinocchio)\" \
                       cargo run -p kmr_ffi_example_pinocchio --features pinocchio --release \
                       -- --dry-run";

    /// Where the build stopped, with what to do about it.
    pub(super) struct Fail(String);

    impl std::fmt::Display for Fail {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            write!(
                f,
                "kmr_ffi_example_pinocchio: {}\n\nWith Pinocchio (https://stack-of-tasks.github.io/\
                 pinocchio/download.html) and pkg-config installed, and the directory holding \
                 pinocchio.pc in PKG_CONFIG_PATH, build with:\n    {RUN}",
                self.0
            )
        }
    }

    /// The whole build.
    pub(super) fn build() -> Result<(), Fail> {
        println!("cargo:rerun-if-env-changed=CXXFLAGS");
        println!("cargo:rerun-if-env-changed=PKG_CONFIG_PATH");
        println!("cargo:rerun-if-env-changed=PINOCCHIO_RPATH");
        let manifest = std::env::var_os("CARGO_MANIFEST_DIR")
            .ok_or_else(|| Fail("CARGO_MANIFEST_DIR is unset; build with cargo".into()))?;
        let out = std::env::var_os("OUT_DIR")
            .ok_or_else(|| Fail("OUT_DIR is unset; build with cargo".into()))?;
        embed(&Path::new(&manifest).join(URDF), Path::new(&out))?;

        let libdir = pkg_config("libdir")?;
        check_cxxflags(&pkg_config("includedir")?)?;
        kmr_api::ffi::c::build::model(URDF)
            .compile(&["src/geckoleg.cpp"])
            .map_err(|e| {
                Fail(format!(
                    "{e}\nThe C++ compile needs Pinocchio's and Eigen's include dirs in CXXFLAGS."
                ))
            })?;

        // DECISION: Pinocchio's libraries after the compile, so their -l follow the static
        // library that uses them: GNU ld with --as-needed (the aarch64 cross link) drops a shared
        // library no earlier object needs; lld, the x86_64 host default, ignores the order.
        println!("cargo:rustc-link-search=native={libdir}");
        println!("cargo:rustc-link-lib=dylib=pinocchio_parsers");
        println!("cargo:rustc-link-lib=dylib=pinocchio_default");
        match std::env::var("PINOCCHIO_RPATH") {
            // DECISION: the given rpath replaces the lib dir's, and the linker still finds the
            // libraries' own dependencies through -rpath-link: in the lib dir, and in the lib64
            // next to it where a pip (cmeel) prefix keeps zlib. scripts/build-pi.sh sets
            // `$ORIGIN/geckoleg_pinocchio-libs/lib` for the libraries deployed next to the binary.
            Ok(rpath) if !rpath.is_empty() => {
                println!("cargo:rustc-link-arg=-Wl,-rpath,{rpath}");
                println!("cargo:rustc-link-arg=-Wl,-rpath-link,{libdir}");
                let lib64 = Path::new(&libdir).join("../lib64");
                if lib64.is_dir() {
                    println!("cargo:rustc-link-arg=-Wl,-rpath-link,{}", lib64.display());
                }
            }
            // DECISION: an rpath to Pinocchio's lib dir, so the binary and the tests run without
            // LD_LIBRARY_PATH from a prefix outside the loader's paths (pip, conda, robotpkg).
            _ => println!("cargo:rustc-link-arg=-Wl,-rpath,{libdir}"),
        }
        Ok(())
    }

    /// Writes `geckoleg_urdf.hpp` into `out`: the URDF at `urdf` as a C++ raw string literal.
    fn embed(urdf: &Path, out: &Path) -> Result<(), Fail> {
        println!("cargo:rerun-if-changed={}", urdf.display());
        let xml = std::fs::read_to_string(urdf)
            .map_err(|e| Fail(format!("cannot read {}: {e}", urdf.display())))?;
        if xml.contains(")urdf\"") {
            return Err(Fail(format!(
                "{} holds `)urdf\"`, the end of the raw string it is embedded in",
                urdf.display()
            )));
        }
        let header = format!(
            "// {}, embedded by build.rs: the model Pinocchio loads. Generated, do not edit.\n\
             #pragma once\n\
             inline constexpr char GECKOLEG_URDF[] = R\"urdf({xml})urdf\";\n",
            urdf.display()
        );
        let path = out.join("geckoleg_urdf.hpp");
        std::fs::write(&path, header)
            .map_err(|e| Fail(format!("cannot write {}: {e}", path.display())))
    }

    /// Pinocchio's pkg-config variable `name`.
    // DECISION: `--variable`, not `--libs`/`--cflags`: pinocchio.pc's `Libs` names no library,
    // and a variable reads without resolving `Requires`, which a pip (cmeel) prefix leaves
    // incomplete (no urdfdom_headers.pc).
    fn pkg_config(name: &str) -> Result<String, Fail> {
        let output = Command::new("pkg-config")
            .args(["--variable", name, "pinocchio"])
            .output()
            .map_err(|e| Fail(format!("cannot run pkg-config: {e}")))?;
        let value = String::from_utf8_lossy(&output.stdout).trim().to_owned();
        if !output.status.success() || value.is_empty() {
            let why = String::from_utf8_lossy(&output.stderr);
            return Err(Fail(format!(
                "pkg-config finds no Pinocchio ({})",
                why.trim().lines().next().unwrap_or("no pinocchio.pc")
            )));
        }
        Ok(value)
    }

    /// Fails unless the C++ compile will see `includedir`: a system dir, or a `-I` of CXXFLAGS
    /// or of its target-scoped form `CXXFLAGS_<target>`, which `cc` reads first.
    fn check_cxxflags(includedir: &str) -> Result<(), Fail> {
        let wanted = canonical(includedir);
        let system = ["/usr/include", "/usr/local/include"].map(canonical);
        if system.contains(&wanted) {
            return Ok(());
        }
        let target = std::env::var("TARGET").unwrap_or_default();
        let scoped = format!("CXXFLAGS_{}", target.replace('-', "_"));
        println!("cargo:rerun-if-env-changed={scoped}");
        // DECISION: accepted from either variable, without mirroring cc's pick of the first one
        // set: the check exists for the error message, the compile itself is cc's.
        let flags = [scoped.as_str(), "CXXFLAGS"]
            .map(|name| std::env::var(name).unwrap_or_default())
            .join(" ");
        let mut words = flags.split_whitespace();
        while let Some(word) = words.next() {
            let dir = match word {
                "-I" | "-isystem" => words.next(),
                _ => word
                    .strip_prefix("-I")
                    .or_else(|| word.strip_prefix("-isystem")),
            };
            if dir.is_some_and(|dir| canonical(dir) == wanted) {
                return Ok(());
            }
        }
        Err(Fail(format!(
            "neither CXXFLAGS nor {scoped} names Pinocchio's include dir {includedir}: kmr's \
             build helper takes no include dir, so the compiler finds Pinocchio through CXXFLAGS"
        )))
    }

    /// `path` with its `..` and links resolved; as given when it does not exist.
    fn canonical(path: &str) -> PathBuf {
        std::fs::canonicalize(path).unwrap_or_else(|_| PathBuf::from(path))
    }
}
