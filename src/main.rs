use image::RgbImage;
use nalgebra::base::{Vector3};
use nalgebra::geometry::Point3;
use std::thread;
use std::sync::{Arc, Mutex};

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

use consts::*;
use material::materials::{Material, Texture};
use hittable::BvhNode;
use geometry::Camera;
use primitive::Primitive;
#[allow(unused_imports)]
use scenes::*;

fn main() {
    let mut img = RgbImage::new(IMAGE_WIDTH, IMAGE_HEIGHT);
    let (camera, node, objs, materials, textures) = scenes::with_everything();

    if SINGLE_THREAD {
        singlethread(&mut img, PATH, &node, &camera, &objs, &materials, &textures);
    } else {
        multithread(img, node, camera, objs, materials, textures);
    }
}

fn singlethread(img: &mut image::RgbImage, path: &str, node: &BvhNode, camera: &Camera, objs: &Vec<Primitive>, materials: &Vec<Material>, textures: &Vec<Texture>) {
    if !COLS {
        let mut pixels: Vec<Vec<(f64, f64, f64, u32)>> = Vec::new();
        for _ in 0..IMAGE_HEIGHT {
            let mut temp: Vec<(f64, f64, f64, u32)> = Vec::new();
            for _ in 0..IMAGE_WIDTH {
                temp.push((0., 0., 0., 0u32));
            }
            pixels.push(temp);
        }

        for r in 0..TOTAL_RAYS {
            if r % 1000000 == 0 {
                println!("Drawing ray {} of {}, {:.2}%", r, TOTAL_RAYS, (r as f64 * 100.) / (TOTAL_RAYS as f64));
            }
            if r % 1000000 == 0 {
                util::draw_picture(img, &pixels, path);
                img.save(path).unwrap();
            }
            let u: f64 = util::rand();
            let v: f64 = util::rand();
            let ray = camera.get_ray(u, v);
            let res = geometry::cast_ray(objs, materials, textures, &ray, &node, MAX_DEPTH);
            let i = (u * IMAGE_WIDTH as f64).floor() as usize;
            let j = (v * IMAGE_HEIGHT as f64).floor() as usize;
            util::increment_color(&mut pixels, j, i, &res);
        }

    } else {
        for i in 0u32..IMAGE_WIDTH {
            println!("Scanning row {} of {}", i, IMAGE_WIDTH);
            for j in 0u32..IMAGE_HEIGHT {
                let mut color: Point3<f64> = Point3::origin();
                for _ in 0u32..SAMPLES_PER_PIXEL {
                    let u: f64 = (i as f64 + util::rand()) / ((IMAGE_WIDTH - 1) as f64);
                    let v: f64 = (j as f64 + util::rand()) / ((IMAGE_HEIGHT - 1) as f64);
                    let ray = camera.get_ray(u, v);
                    let res: Vector3<f64> = geometry::cast_ray(objs, materials, textures, &ray, &node, MAX_DEPTH);
                    color = color + res;
                }
                util::draw_color(img, i, j, &color, SAMPLES_PER_PIXEL);
            }
            if i % 50 == 0 {
                img.save(path).unwrap();
            }
        }
    }
    img.save(path).unwrap();
}

fn multithread (img: image::RgbImage, node: BvhNode, camera: Camera, objs: Vec<Primitive>, materials: Vec<Material>, textures: Vec<Texture>) {
    let pixels: Vec<Vec<(f64, f64, f64, u32)>> = util::make_empty_image();

    let pixels_mutex: Arc<Mutex<Vec<Vec<(f64, f64, f64, u32)>>>> = Arc::new(Mutex::new(pixels));
    let node_arc: Arc<BvhNode> = Arc::new(node);

    let image_arc: Arc<Mutex<image::RgbImage>> = Arc::new(Mutex::new(img));

    let mut thread_vec: Vec<thread::JoinHandle<()>> = Vec::new();
    let objs_arc: Arc<Vec<Primitive>> = Arc::new(objs);
    let mats_arc: Arc<Vec<Material>> = Arc::new(materials);
    let texture_arc : Arc<Vec<Texture>> = Arc::new(textures);

    for thread_num in 0..NUM_THREADS {
        let camera_clone = camera.clone();
        let node_clone = Arc::clone(&node_arc);
        let pixels_clone = Arc::clone(&pixels_mutex);
        let image_clone = Arc::clone(&image_arc);
        let objs_clone= Arc::clone(&objs_arc);
        let mats_clone = Arc::clone(&mats_arc);
        let textures_clone = Arc::clone(&texture_arc);

        thread_vec.push(thread::spawn(move || {
            let mut local_img: Vec<Vec<(f64, f64, f64, u32)>> = util::make_empty_image();
            for r in 0..RAYS_PER_THREAD {
                if r % THREAD_UPDATE == 0 {
                    println!("Thread {} drawing ray {} of {} ({:.2}%)", thread_num, r, RAYS_PER_THREAD, (r as f64 * 100.) / (RAYS_PER_THREAD as f64));
                    if thread_num == 0 { // only draw once
                        util::thread_safe_draw_picture(&image_clone, &pixels_clone, PATH);
                    }
                    util::thread_safe_update_image(&pixels_clone, &local_img);
                }
                let u: f64 = util::rand();
                let v: f64 = util::rand();
                let ray = camera_clone.get_ray(u, v);
                let res = geometry::cast_ray(&objs_clone, &mats_clone, &textures_clone, &ray, &node_clone, MAX_DEPTH);
                let i = (u * IMAGE_WIDTH as f64).floor() as usize;
                let j = (v * IMAGE_HEIGHT as f64).floor() as usize;
                local_img[j][i].0 += res.x;
                local_img[j][i].1 += res.y;
                local_img[j][i].2 += res.z;
                local_img[j][i].3 += 1;
                // util::thread_safe_increment_color(&pixels_clone, j, i, &res);
            }
        }));
    }

    for handle in thread_vec {
        handle.join().unwrap();
    }

    image_arc.lock().unwrap().save(PATH).unwrap();
}