pub const INFINITY: f64 = 1e308;
pub const PI: f64 = 3.14159265358979;
pub const SMALL: f64 = 0.001;
pub const GAMMA: f64 = 1.5;
pub const IMAGE_WIDTH: u32 = 1920;
pub const ASPECT_RATIO: f64 = 16.0 / 9.0;
pub const IMAGE_HEIGHT: u32 = ((IMAGE_WIDTH as f64) / ASPECT_RATIO) as u32;
pub const SAMPLES_PER_PIXEL: u32 = 20;
pub const MAX_DEPTH: u32 = 15;
pub const COLS: bool = false;
pub const SINGLE_THREAD: bool = false;
pub const NUM_THREADS: u32 = 6;
pub const TOTAL_RAYS: u32 = IMAGE_WIDTH * IMAGE_HEIGHT * SAMPLES_PER_PIXEL;
pub const RAYS_PER_THREAD: u32 = TOTAL_RAYS / NUM_THREADS;
pub const THREAD_UPDATE: u32 = RAYS_PER_THREAD / 20u32;
pub const PATH: &str = "test2.png";