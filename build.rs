use std::{fs, io};

fn main() -> io::Result<()> {
    println!("cargo:rerun-if-changed=dmsrc/navmap_pathfinder.dm");

    fs::create_dir_all("target")?;
    let dm = fs::read_to_string("dmsrc/navmap_pathfinder.dm")?;
    fs::write("target/navmap_pathfinder.dm", dm)
}
