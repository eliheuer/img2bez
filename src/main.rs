use clap::Parser;
use img2bez::TracingConfig;
use std::path::PathBuf;

// Re-used for direct UFO save (previously done inside trace_into_ufo).
use norad;

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

    /// Advance width (auto-computed if omitted)
    #[arg(short = 'w', long)]
    width: Option<f64>,

    /// Grid size for coordinate snapping (0 = off)
    #[arg(long, default_value = "0")]
    grid: i32,

    /// Chamfer size (0 = off)
    #[arg(long, default_value = "0")]
    chamfer: f64,

    /// Curve fitting accuracy in font units
    #[arg(long, default_value = "4.0")]
    accuracy: f64,

    /// RDP simplification epsilon in font units
    #[arg(long, default_value = "8.0")]
    rdp_epsilon: f64,

    /// Corner detection threshold in degrees
    #[arg(long, default_value = "30")]
    corner_threshold: f64,

    /// Polygon smoothing iterations before curve fitting (0 = off)
    #[arg(long, default_value = "3")]
    smooth: usize,

    /// Corner detection sensitivity (0.0–1.34)
    /// Lower = more corners = shorter smooth sections = tighter fits.
    /// Try 0.6–0.8 for geometric type, 1.0 for organic shapes.
    #[arg(long, default_value = "1.0")]
    alphamax: f64,

    /// Target height in font units
    #[arg(long, default_value = "1000")]
    target_height: f64,

    /// Y offset after scaling (typically descender)
    #[arg(long, default_value = "0", allow_hyphen_values = true)]
    y_offset: f64,

    /// Invert the image before tracing
    #[arg(long)]
    invert: bool,

    /// Reference .glif file for quality evaluation
    #[arg(long)]
    reference: Option<PathBuf>,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let cli = Cli::parse();

    let codepoints = match &cli.unicode {
        Some(u) => {
            let cp = u32::from_str_radix(u, 16)?;
            char::from_u32(cp).into_iter().collect()
        }
        None => vec![],
    };

    let config = TracingConfig {
        grid: cli.grid,
        chamfer_size: cli.chamfer,
        fit_accuracy: cli.accuracy,
        rdp_epsilon: cli.rdp_epsilon,
        corner_angle_threshold: cli.corner_threshold.to_radians(),
        smooth_iterations: cli.smooth,
        alphamax: cli.alphamax,
        advance_width: cli.width,
        target_height: cli.target_height,
        y_offset: cli.y_offset,
        invert: cli.invert,
        codepoints,
        ..TracingConfig::default()
    };

    // Header
    let unicode_str = cli.unicode.as_deref()
        .map(|u| format!(" (U+{})", u.to_uppercase()))
        .unwrap_or_default();
    eprintln!();
    eprintln!("  img2bez \u{00b7} {}{}", cli.name, unicode_str);
    eprintln!();

    // Pipeline (lib prints step-by-step progress to stderr)
    let result = img2bez::trace(&cli.input, &config)?;
    let glyph = img2bez::ufo::to_glyph(&cli.name, &result, &config)?;
    let mut font = norad::Font::load(&cli.output)?;
    font.default_layer_mut().insert_glyph(glyph);
    font.save(&cli.output)?;

    // Footer
    eprintln!();
    eprintln!("  \u{2713} {}", cli.output.display());

    // Optional reference comparison
    if let Some(ref_path) = &cli.reference {
        let traced = img2bez::eval::from_trace_result(&result);
        let reference = img2bez::eval::load_glif(ref_path)?;
        let report = img2bez::eval::evaluate(&traced, &reference, cli.grid, &ref_path.display().to_string());
        eprint!("{}", report);
    }

    eprintln!();

    Ok(())
}
