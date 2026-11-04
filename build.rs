use std::{fs, io};

fn main() -> io::Result<()> {
    println!("cargo:rerun-if-changed=dmsrc/navmap_pathfinder.dm");

    fs::create_dir_all("target")?;
    let dm = fs::read_to_string("dmsrc/navmap_pathfinder.dm")?.replace(
        "@CARGO_PKG_VERSION@",
        &std::env::var("CARGO_PKG_VERSION")
            .expect("Cargo sets CARGO_PKG_VERSION for build scripts"),
    );
    fs::write("target/navmap_pathfinder.dm", dm)
}
