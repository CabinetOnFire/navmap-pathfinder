use std::{fs, io, path::Path};

fn append_file(output: &mut String, path: impl AsRef<Path>) -> io::Result<()> {
    output.push_str(&fs::read_to_string(path)?);
    output.push('\n');
    Ok(())
}

fn main() -> io::Result<()> {
    println!("cargo:rerun-if-changed=dmsrc/navmap_pathfinder.dm");

    let mut dm = String::new();
    append_file(&mut dm, "dmsrc/navmap_pathfinder.dm")?;

    fs::create_dir_all("target")?;
    fs::write("target/navmap_pathfinder.dm", dm)
}
