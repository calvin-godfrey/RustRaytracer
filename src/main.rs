use std::time::SystemTime;
use consts::*;
use integrator::IntType;
use scenes::*;

mod util;
mod consts;
mod material;
mod hittable;
mod geometry;
mod parser;
mod scenes;
mod perlin;
mod primitive;
mod intersects;
mod bxdf;
mod microfacet;
mod bsdf;
mod light;
mod sampler;
mod integrator;
mod distribution;
mod render;

fn main() {
    let now = SystemTime::now();
    let int_type = IntType::Path { max_depth: MAX_DEPTH, invisible_light: false };
    let (path, camera, sampler) = material_hdr();
    render::tile_multithread(path, camera, sampler, int_type);
    match now.elapsed() {
        Ok(elapsed) => {
            let milliseconds = elapsed.as_millis() % 1000;
            let seconds = elapsed.as_secs();
            let minutes = seconds / 60;
            let hours = minutes / 60;
            let seconds = seconds % 60;
            if hours != 0 {
                let minutes = minutes - hours * 60;
                println!("Render finished after {} ms ({}:{}:{}.{})", elapsed.as_millis(), hours, minutes, seconds, milliseconds);
            } else {
                if minutes == 0 {
                    println!("Render finished after {} ms ({}.{}s)", elapsed.as_millis(), seconds, milliseconds);
                } else {
                    println!("Render finished after {} ms ({}:{}.{})", elapsed.as_millis(), minutes, seconds, milliseconds);
                }
            }
        }
        Err(_) => {}
    }
}
