pub const INFINITY: f64 = 1e308;
pub const PI: f64 = 3.14159265358979;
pub const SMALL: f64 = 0.001;
pub const GAMMA: f64 = 1.5;
pub const IMAGE_WIDTH: u32 = 1920;
pub const ASPECT_RATIO: f64 = 16.0 / 9.0;
pub const IMAGE_HEIGHT: u32 = ((IMAGE_WIDTH as f64) / ASPECT_RATIO) as u32;
pub const SAMPLES_PER_PIXEL: u32 = 250;
pub const MAX_DEPTH: u32 = 25;
pub const COLS: bool = true;
pub const SINGLE_THREAD: bool = false;
pub const NUM_THREADS: u32 = 6;
pub const TOTAL_RAYS: u64 = (IMAGE_WIDTH as u64) * (IMAGE_HEIGHT as u64) * (SAMPLES_PER_PIXEL as u64);
pub const RAYS_PER_THREAD: u64 = TOTAL_RAYS / (NUM_THREADS as u64);
pub const THREAD_UPDATE: u64 = RAYS_PER_THREAD / 50u64;
pub const PATH: &str = "test3.png";
pub const INV_COL_MAX: f64 = 1. / 256.;
pub const AMBIENT_LIGHT: bool = true;