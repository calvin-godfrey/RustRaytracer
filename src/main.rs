use image::RgbImage;
use std::{collections::HashSet, sync::{Arc, Mutex}, thread, time::SystemTime};
use consts::*;
use integrator::IntType;
use crate::geometry::Camera;
use crate::sampler::Samplers;
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

fn main() {
    let now = SystemTime::now();
    let (path, camera, sampler, int_type) = two_dragons();
    tile_multithread(path, camera, sampler, int_type);
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

fn tile_multithread(path: String, camera: Camera, sampler: Samplers, int_type: IntType) {
    let img = RgbImage::new(IMAGE_WIDTH, IMAGE_HEIGHT);
    let pixels: Vec<Vec<(f64, f64, f64, u32)>> = util::make_empty_image(IMAGE_HEIGHT as usize, IMAGE_WIDTH as usize);
    let pixels_mutex: Arc<Mutex<Vec<Vec<(f64, f64, f64, u32)>>>> = Arc::new(Mutex::new(pixels));
    let image_arc: Arc<Mutex<image::RgbImage>> = Arc::new(Mutex::new(img));
    let mut thread_vec: Vec<thread::JoinHandle<()>> = Vec::new();

    // START TILE STUFF
    let num_tiles_wide = (IMAGE_WIDTH) / TILE_WIDTH + 1;
    let num_tiles_tall = (IMAGE_HEIGHT) / TILE_HEIGHT + 1;
    let mut started_tiles: Vec<Vec<i32>> = Vec::new();
    for i in 0..num_tiles_tall {
        started_tiles.push(Vec::new());
        for _ in 0..num_tiles_wide {
            started_tiles[i as usize].push(-1);
        }
    }
    let tiles_arc: Arc<Mutex<Vec<Vec<i32>>>> = Arc::new(Mutex::new(started_tiles));

    for _ in 0..NUM_THREADS {
        let camera_clone = camera.clone();
        let sampler_clone = sampler.clone();
        let int_type_clone = int_type.clone();
        let pixels_clone = Arc::clone(&pixels_mutex);
        let tiles_clone = Arc::clone(&tiles_arc);

        thread_vec.push(thread::spawn(move || {
            let mut img_integrator = integrator::get_integrator(int_type_clone, camera_clone, sampler_clone);
            img_integrator.init();
            loop {
                let mut tiles = tiles_clone.lock().unwrap();
                let mut tx: u32 = 0;
                let mut ty: u32 = 0;
                let mut done = false;
                
                // look for a tile that hasn't been started yet
                'outer: for i in 0..tiles.len() {
                    for j in 0..tiles[i].len() {
                        if tiles[i][j] == -1 {
                            // if a tile hasn't been started, record its data and mark it as started
                            tx = j as u32;
                            ty = i as u32;
                            tiles[i][j] = 0;
                            done = true;
                            break 'outer;
                        }
                    }
                }
                if !done { // all tiles are taken
                    break;
                }
                drop(tiles); // no longer need to hold the lock
                let mut local_img: Vec<Vec<(f64, f64, f64, u32)>> = util::make_empty_image(16, 16);
                for y in 0..TILE_HEIGHT {
                    for x in 0..TILE_WIDTH {
                        let px = tx * TILE_WIDTH + x;
                        let py = ty * TILE_HEIGHT + y;
                        if py >= IMAGE_HEIGHT {
                            continue;
                        }
                        if px >= IMAGE_WIDTH {
                            break;
                        }


                        img_integrator.render(&mut local_img, px, py);
                        let finished_pixel = local_img[y as usize][x as usize];
                        util::thread_safe_write_pixel(&pixels_clone, py as usize, px as usize, finished_pixel);
                    }
                }
                // acquire tiles lock and mark tile as completed
                let mut after_tiles = tiles_clone.lock().unwrap();
                after_tiles[ty as usize][tx as usize] = 1;
                drop(after_tiles);
            }
        }));
    }

    let img_clone = Arc::clone(&image_arc);
    let tile_progress = Arc::clone(&tiles_arc);

    // this thread just checks every five seconds to update the displayed picture
    thread_vec.push(thread::spawn(move || {
        let mut local_img = util::make_empty_image(IMAGE_HEIGHT as usize, IMAGE_WIDTH as usize);
        let mut changed_tiles: HashSet<(usize, usize)>;
        loop {
            thread::sleep(std::time::Duration::from_secs(UPDATE_PICTURE_FREQUENCY));
            changed_tiles = HashSet::new(); // reset the tiles
            let tiles = tile_progress.lock().unwrap();
            for i in 0..tiles.len() {
                for j in 0..tiles[i].len() {
                    if tiles[i][j] == 0 { // in progress
                        changed_tiles.insert((j, i));
                    }
                }
            }
            let pixels_guard = pixels_mutex.lock().unwrap();
            for i in 0..IMAGE_HEIGHT {
                for j in 0..IMAGE_WIDTH {
                    if local_img[i as usize][j as usize] != pixels_guard[i as usize][j as usize] {
                        local_img[i as usize][j as usize] = pixels_guard[i as usize][j as usize];
                    }
                }
            }
            drop(pixels_guard);
            drop(tiles);
            util::thread_safe_draw_picture(&img_clone, &pixels_mutex, &changed_tiles, &path[..]);
            if changed_tiles.len() == 0 { // none of the pixels have changed, so we're done
                break;
            }
        }
    }));

    for handle in thread_vec {
        handle.join().unwrap();
    }
}