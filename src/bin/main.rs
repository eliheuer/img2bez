use clap::Parser;
use img2bez::TracingConfig;
use std::path::PathBuf;

#[derive(Parser)]
#[command(name = "img2bez", about = "Bitmap image to font-ready bezier contours")]
struct Cli {
    /// Input image path (PNG, JPEG, BMP)
    #[arg(short, long)]
    input: PathBuf,

    /// Output UFO path (will insert/replace the glyph)
    #[arg(short, long)]
    output: PathBuf,

    /// Glyph name
    #[arg(short, long)]
    name: String,

    /// Unicode codepoint (hex, e.g. "003F" for ?)
    #[arg(short = 'u', long)]
    unicode: Option<String>,

    /// Advance width (auto-computed from bounds if omitted)
    #[arg(short = 'w', long)]
    width: Option<f64>,

    /// Grid size for coordinate snapping (0 = off)
    #[arg(long, default_value = "0")]
    grid: i32,

    /// Chamfer size (0 = off)
    #[arg(long, default_value = "0")]
    chamfer: f64,

    /// Curve fitting accuracy in font units (smaller = more precise, more points)
    #[arg(long, default_value = "8.0")]
    accuracy: f64,

    /// RDP simplification epsilon in font units (higher = fewer input points)
    #[arg(long, default_value = "8.0")]
    rdp_epsilon: f64,

    /// Corner detection angle threshold in degrees
    #[arg(long, default_value = "30")]
    corner_threshold: f64,

    /// Target height in font units (image height maps to this)
    #[arg(long, default_value = "1000")]
    target_height: f64,

    /// Y offset after scaling (typically the descender value)
    #[arg(long, default_value = "0", allow_hyphen_values = true)]
    y_offset: f64,

    /// Invert the image before tracing
    #[arg(long)]
    invert: bool,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let cli = Cli::parse();

    let mut config = TracingConfig::default();
    config.grid = cli.grid;
    config.chamfer_size = cli.chamfer;
    config.fit_accuracy = cli.accuracy;
    config.rdp_epsilon = cli.rdp_epsilon;
    config.corner_angle_threshold = cli.corner_threshold.to_radians();
    config.advance_width = cli.width;
    config.target_height = cli.target_height;
    config.y_offset = cli.y_offset;
    config.invert = cli.invert;

    if let Some(ref u) = cli.unicode {
        let cp = u32::from_str_radix(u, 16)?;
        if let Some(c) = char::from_u32(cp) {
            config.codepoints = vec![c];
        }
    }

    img2bez::trace_into_ufo(&cli.input, &cli.name, &cli.output, &config)?;

    eprintln!("Traced '{}' into {}", cli.name, cli.output.display());
    Ok(())
}
