use crate::geometry::Camera;
use crate::integrator::{get_integrator, IntType};
use crate::sampler::Samplers;
use crate::util;
use crate::{consts::*, GLOBAL_STATE};
use image::RgbImage;
use std::{
    collections::HashSet,
    sync::{atomic::Ordering, Arc, Mutex},
    thread,
};

pub fn tile_multithread(path: String, camera: Camera, sampler: Samplers, int_type: IntType) {
    let img = RgbImage::new(
        GLOBAL_STATE.get_image_width(),
        GLOBAL_STATE.get_image_height(),
    );
    let pixels: Vec<Vec<(f64, f64, f64, u32)>> = util::make_empty_image(
        GLOBAL_STATE.get_image_height() as usize,
        GLOBAL_STATE.get_image_width() as usize,
    );
    let pixels_mutex: Arc<Mutex<Vec<Vec<(f64, f64, f64, u32)>>>> = Arc::new(Mutex::new(pixels));
    let image_arc: Arc<Mutex<image::RgbImage>> = Arc::new(Mutex::new(img));
    let mut thread_vec: Vec<thread::JoinHandle<()>> = Vec::new();

    // START TILE STUFF
    let num_tiles_wide = (GLOBAL_STATE.get_image_width()) / TILE_SIZE + 1;
    let num_tiles_tall = (GLOBAL_STATE.get_image_height()) / TILE_SIZE + 1;
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
            let mut img_integrator = get_integrator(int_type_clone, camera_clone, sampler_clone);
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
                if !done {
                    // all tiles are taken
                    break;
                }
                drop(tiles); // no longer need to hold the lock
                let mut local_img: Vec<Vec<(f64, f64, f64, u32)>> = util::make_empty_image(16, 16);
                let mut done = false;
                for y in 0..TILE_SIZE {
                    for x in 0..TILE_SIZE {
                        let px = tx * TILE_SIZE + x;
                        let py = ty * TILE_SIZE + y;
                        if py >= GLOBAL_STATE.get_image_height() {
                            continue;
                        }
                        if px >= GLOBAL_STATE.get_image_width() {
                            break;
                        }

                        img_integrator.render(&mut local_img, px, py);
                        let finished_pixel = local_img[y as usize][x as usize];
                        util::thread_safe_write_pixel(
                            &pixels_clone,
                            py as usize,
                            px as usize,
                            finished_pixel,
                        );
                        if GLOBAL_STATE.stop_render.load(Ordering::Relaxed) {
                            done = true;
                            break;
                        }
                    }
                    if done {
                        break;
                    }
                }
                if done {
                    break;
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
        let mut local_img = util::make_empty_image(
            GLOBAL_STATE.get_image_height() as usize,
            GLOBAL_STATE.get_image_width() as usize,
        );
        let mut changed_tiles: HashSet<(usize, usize)>;
        loop {
            thread::sleep(std::time::Duration::from_millis(
                (UPDATE_PICTURE_FREQUENCY * 1000.0) as u64,
            ));
            changed_tiles = HashSet::new(); // reset the tiles
            let tiles = tile_progress.lock().unwrap();
            for i in 0..tiles.len() {
                for j in 0..tiles[i].len() {
                    if tiles[i][j] == 0 {
                        // in progress
                        changed_tiles.insert((j, i));
                    }
                }
            }
            let pixels_guard = pixels_mutex.lock().unwrap();
            for i in 0..GLOBAL_STATE.get_image_height() {
                for j in 0..GLOBAL_STATE.get_image_width() {
                    if local_img[i as usize][j as usize] != pixels_guard[i as usize][j as usize] {
                        local_img[i as usize][j as usize] = pixels_guard[i as usize][j as usize];
                    }
                }
            }
            drop(pixels_guard);
            drop(tiles);
            util::thread_safe_draw_picture(&img_clone, &pixels_mutex, &changed_tiles, &path[..]);
            if changed_tiles.len() == 0 || GLOBAL_STATE.stop_render.load(Ordering::Relaxed) {
                // none of the pixels have changed, so we're done
                break;
            }
        }
    }));

    for handle in thread_vec {
        handle.join().unwrap();
    }
    GLOBAL_STATE.stop_render.store(false, Ordering::Relaxed); // let the main thread know that the rendering is done
}

pub fn progressive_multithread(path: String, camera: Camera, sampler: Samplers, int_type: IntType) {
    let img = RgbImage::new(
        GLOBAL_STATE.get_image_width(),
        GLOBAL_STATE.get_image_height(),
    );
    let pixels: Vec<Vec<(f64, f64, f64, u32)>> = util::make_empty_image(
        GLOBAL_STATE.get_image_height() as usize,
        GLOBAL_STATE.get_image_width() as usize,
    );
    let pixels_mutex: Arc<Mutex<Vec<Vec<(f64, f64, f64, u32)>>>> = Arc::new(Mutex::new(pixels));
    let image_arc: Arc<Mutex<image::RgbImage>> = Arc::new(Mutex::new(img));
    let mut thread_vec: Vec<thread::JoinHandle<()>> = Vec::new();

    // do a single ray tile-wise to make sure every pixel gets hit at least once
    let num_tiles_wide = (GLOBAL_STATE.get_image_width()) / TILE_SIZE + 1;
    let num_tiles_tall = (GLOBAL_STATE.get_image_height()) / TILE_SIZE + 1;
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
            let mut img_integrator = get_integrator(int_type_clone, camera_clone, sampler_clone);
            img_integrator.init();
            let mut local_img: Vec<Vec<(f64, f64, f64, u32)>>;
            let mut stop = false;

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
                if !done {
                    // all tiles are taken
                    break;
                }
                drop(tiles); // no longer need to hold the lock
                let mut local_img: Vec<Vec<(f64, f64, f64, u32)>> = util::make_empty_image(16, 16);
                for y in 0..TILE_SIZE {
                    for x in 0..TILE_SIZE {
                        stop = GLOBAL_STATE.stop_render.load(Ordering::Relaxed);
                        if stop {
                            break;
                        }
                        let px = tx * TILE_SIZE + x;
                        let py = ty * TILE_SIZE + y;
                        if py >= GLOBAL_STATE.get_image_height() {
                            continue;
                        }
                        if px >= GLOBAL_STATE.get_image_width() {
                            break;
                        }

                        img_integrator.single_sample(&mut local_img, px, py);
                        let finished_pixel = local_img[y as usize][x as usize];
                        util::thread_safe_write_pixel(
                            &pixels_clone,
                            py as usize,
                            px as usize,
                            finished_pixel,
                        );
                    }
                    if stop {
                        break;
                    }
                }
                // acquire tiles lock and mark tile as completed
                let mut after_tiles = tiles_clone.lock().unwrap();
                after_tiles[ty as usize][tx as usize] = 1;
                drop(after_tiles);
                if stop {
                    break;
                }
            }
            local_img = util::make_empty_image(
                GLOBAL_STATE.get_image_height() as usize,
                GLOBAL_STATE.get_image_width() as usize,
            );
            if !stop {
                for ray_num in 0..((GLOBAL_STATE.get_image_width() as u64)
                    * (GLOBAL_STATE.get_image_height() as u64)
                    * (GLOBAL_STATE.get_samples_per_pixel() as u64))
                {
                    let px = (util::rand() * GLOBAL_STATE.get_image_width() as f64) as u32;
                    let py = (util::rand() * GLOBAL_STATE.get_image_height() as f64) as u32;
                    img_integrator.single_sample(&mut local_img, px, py);
                    if ray_num % THREAD_UPDATE == 0 {
                        util::thread_safe_update_image(&pixels_clone, &local_img);
                        local_img = util::make_empty_image(
                            GLOBAL_STATE.get_image_height() as usize,
                            GLOBAL_STATE.get_image_width() as usize,
                        );
                    }
                    if GLOBAL_STATE.stop_render.load(Ordering::Relaxed) {
                        // static AtomicBool in main
                        break;
                    }
                }
            }
        }));
    }

    let img_clone = Arc::clone(&image_arc);

    // this thread just checks every five seconds to update the displayed picture
    thread_vec.push(thread::spawn(move || {
        let mut local_img = util::make_empty_image(
            GLOBAL_STATE.get_image_height() as usize,
            GLOBAL_STATE.get_image_width() as usize,
        );
        loop {
            let mut update = false;
            thread::sleep(std::time::Duration::from_millis(
                (UPDATE_PICTURE_FREQUENCY * 1000.0) as u64,
            ));
            let pixels_guard = pixels_mutex.lock().unwrap();
            for i in 0..GLOBAL_STATE.get_image_height() {
                for j in 0..GLOBAL_STATE.get_image_width() {
                    if local_img[i as usize][j as usize] != pixels_guard[i as usize][j as usize] {
                        local_img[i as usize][j as usize] = pixels_guard[i as usize][j as usize];
                        update = true;
                    }
                }
            }
            drop(pixels_guard);
            util::progressive_draw_picture(&img_clone, &pixels_mutex, &path[..]);
            if !update || GLOBAL_STATE.stop_render.load(Ordering::Relaxed) {
                // none of the pixels have changed, so we're done
                break;
            }
        }
    }));

    for handle in thread_vec {
        handle.join().unwrap();
    }

    GLOBAL_STATE.stop_render.store(false, Ordering::Relaxed);
}
