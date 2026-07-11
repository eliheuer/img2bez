// Embed the img2bez repo's git revision so every sheet records which
// tracer build drew it (the tool compiles the library from the checkout).
use std::process::Command;

fn main() {
    let manifest = std::env::var("CARGO_MANIFEST_DIR").unwrap_or_default();
    let repo = format!("{manifest}/../..");
    let rev = Command::new("git")
        .args(["-C", &repo, "rev-parse", "--short", "HEAD"])
        .output()
        .ok()
        .filter(|o| o.status.success())
        .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
        .unwrap_or_else(|| "unknown".into());
    let dirty = Command::new("git")
        .args(["-C", &repo, "status", "--porcelain"])
        .output()
        .ok()
        .filter(|o| o.status.success())
        .map(|o| !o.stdout.is_empty())
        .unwrap_or(false);
    println!(
        "cargo:rustc-env=IMG2BEZ_GIT_REV={rev}{}",
        if dirty { "+dirty" } else { "" }
    );
    // .git/HEAD only holds the branch NAME; the commit hash lives in the
    // branch ref, so watch both or the stamp goes stale on new commits.
    println!("cargo:rerun-if-changed={repo}/.git/HEAD");
    println!("cargo:rerun-if-changed={repo}/.git/refs/heads/main");
    println!("cargo:rerun-if-changed={repo}/src");
}
