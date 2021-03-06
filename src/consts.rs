#![allow(dead_code)]
// Defining rendering quantities
pub const GAMMA: f64 = 2.2;
// pub const IMAGE_WIDTH: u32 = 720;
// pub const ASPECT_RATIO: f64 = 9.0 / 9.0;
// pub const SAMPLES_PER_PIXEL: u32 = 1000;
pub const MAX_DEPTH: u32 = 25;
pub const NUM_THREADS: u32 = 6;
pub const AMBIENT_LIGHT: bool = true;
pub const TILE_SIZE: u32 = 16;
pub const UPDATE_PICTURE_FREQUENCY: f64 = 0.05;
pub const TONE_MAPPING: bool = true;
// BxDF type constants
pub const BSDF_REFLECTION: u8 = 1 << 0;
pub const BSDF_TRANSMISSION: u8 = 1 << 1;
pub const BSDF_DIFFUSE: u8 = 1 << 2;
pub const BSDF_GLOSSY: u8 = 1 << 3;
pub const BSDF_SPECULAR: u8 = 1 << 4;
pub const BSDF_ALL: u8 =
    BSDF_REFLECTION | BSDF_TRANSMISSION | BSDF_DIFFUSE | BSDF_GLOSSY | BSDF_SPECULAR;
// constants for light types
pub const DELTA_POSITION: u8 = 1 << 0;
pub const DELTA_DIRECTION: u8 = 1 << 1;
pub const AREA: u8 = 1 << 2;
pub const INFINITE: u8 = 1 << 3;
// constants for if ray represents light or importance (start at light or camera)
pub const IMPORTANCE: u8 = 1;
pub const RADIANCE: u8 = 0;
// Universal Consts
pub const INFINITY: f64 = 1e308;
pub const PI: f64 = 3.14159265358979;
pub const SMALL: f64 = 0.001;
pub const INV_COL_MAX: f64 = 1. / 256.;
pub const ONE_MINUS_EPSILON: f64 = 1f64 - std::f64::EPSILON / 2.;
// Defined in terms of those above
// pub const IMAGE_HEIGHT: u32 = ((IMAGE_WIDTH as f64) / ASPECT_RATIO) as u32;
pub const THREAD_UPDATE: u64 = 500;
// pub const TOTAL_RAYS: u64 =
//     (IMAGE_WIDTH as u64) * (IMAGE_HEIGHT as u64) * (SAMPLES_PER_PIXEL as u64);
// pub const RAYS_PER_THREAD: u64 = TOTAL_RAYS / (NUM_THREADS as u64);
pub const INV_PI: f64 = 1f64 / PI;
pub const INV_2PI: f64 = 1f64 / (2f64 * PI);
