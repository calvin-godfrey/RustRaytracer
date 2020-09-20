#![allow(dead_code)]
// Universal Consts
pub const INFINITY: f64 = 1e308;
pub const PI: f64 = 3.14159265358979;
pub const SMALL: f64 = 0.001;
pub const INV_COL_MAX: f64 = 1. / 256.;
// Defining rendering quantities
pub const GAMMA: f64 = 1.8;
pub const IMAGE_WIDTH: u32 = 240;
pub const ASPECT_RATIO: f64 = 9.0 / 9.0;
pub const SAMPLES_PER_PIXEL: u32 = 1000;
pub const MAX_DEPTH: u32 = 50;
pub const COLS: bool = true;
pub const SINGLE_THREAD: bool = false;
pub const NUM_THREADS: u32 = 8;
pub const AMBIENT_LIGHT: bool = false;
pub const TILED: bool = true;
pub const TILE_WIDTH: u32 = 16;
pub const TILE_HEIGHT: u32 = 16;
pub const UPDATE_PICTURE_FREQUENCY: u64 = 5;
// BxDF type constants
pub const BSDF_REFLECTION: u8 = 1 << 0;
pub const BSDF_TRANSMISSION: u8 = 1 << 1;
pub const BSDF_DIFFUSE: u8 = 1 << 2;
pub const BSDF_GLOSSY: u8 = 1 << 3;
pub const BSDF_SPECULAR: u8 = 1 << 4;
pub const BSDF_ALL: u8 = BSDF_REFLECTION | BSDF_TRANSMISSION | BSDF_DIFFUSE | BSDF_GLOSSY | BSDF_SPECULAR;
// constants for if ray represents light or importance (start at light or camera)
pub const IMPORTANCE: u8 = 1;
pub const RADIANCE: u8 = 0;
// Defined in terms of those above
pub const IMAGE_HEIGHT: u32 = ((IMAGE_WIDTH as f64) / ASPECT_RATIO) as u32;
pub const THREAD_UPDATE: u64 = RAYS_PER_THREAD / 200u64;
pub const TOTAL_RAYS: u64 = (IMAGE_WIDTH as u64) * (IMAGE_HEIGHT as u64) * (SAMPLES_PER_PIXEL as u64);
pub const RAYS_PER_THREAD: u64 = TOTAL_RAYS / (NUM_THREADS as u64);
pub const INV_PI: f64 = 1f64 / PI;