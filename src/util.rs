use image::{Rgb, RgbImage};
use nalgebra::geometry::{Projective3, Point3};
use nalgebra::base::{Unit, Vector3, Vector2};
use rand::prelude::*;
use rand::distributions::Standard;
use std::sync::{Mutex, Arc};

use crate::consts::*;
use crate::geometry;
use crate::hittable;
use crate::primitive::Primitive;

pub fn gradient(from: &Rgb<u8>, to: &Rgb<u8>, scale: f64) -> Rgb<u8> {
    let r: u8 = ((1.0 - scale) * from[0] as f64 + (scale * (to[0] as f64))) as u8;
    let g: u8 = ((1.0 - scale) * from[1] as f64 + (scale * (to[1] as f64))) as u8;
    let b: u8 = ((1.0 - scale) * from[2] as f64 + (scale * (to[2] as f64))) as u8;
    Rgb([r, g, b])
}

#[allow(dead_code)]
pub fn clamp(x: f64, min: f64, max: f64) -> f64 {
    if x < min {
        return min;
    } else if x > max {
        return max;
    }
    return x;
}

pub fn rand() -> f64 {
    StdRng::from_entropy().sample(Standard)
}

pub fn rand_range(min: f64, max: f64) -> f64 {
    return min + (max - min) * rand();
}

pub fn draw_color(img: &mut RgbImage, i: u32, j: u32, color: &Point3<f64>, samples: u32) {
    let ans = point_to_color(color, 1. / GAMMA, samples);
    img.put_pixel(i, j, ans);
}

pub fn rand_in_unit_sphere() -> Vector3<f64> {
    let a = rand_range(0.,2. * PI);
    let z = rand_range(-1., 1.);
    let r = (1. - z * z).sqrt();
    return Vector3::new(r * a.cos(), r * a.sin(), z);
}

#[allow(dead_code)]
pub fn rand_in_hemisphere(normal: &Vector3<f64>) -> Vector3<f64> {
    let vec: Vector3<f64> = rand_in_unit_sphere();
    if normal.dot(&vec) > 0. {
        vec
    } else {
        -vec
    }
}

pub fn rand_in_disk() -> Vector3<f64> {
    loop {
        let x = rand();
        let y = rand();
        if x*x + y*y < 1. {
            return Vector3::new(x, y, 0.);
        }
    }
}

pub fn rand_int(min: i32, max: i32) -> i32 {
    rand_range(min as f64, max as f64).round() as i32
}

pub fn rand_vector(min: f64, max: f64) -> Vector3<f64> {
    Vector3::new(rand_range(min, max), rand_range(min, max), rand_range(min, max))
}

pub fn reflect(v: &Vector3<f64>, n: &Unit<Vector3<f64>>) -> Vector3<f64> {
    let scale = 2. * v.dot(n);
    v - n.as_ref().scale(scale)
}

pub fn increment_color(arr: &mut Vec<Vec<(f64, f64, f64, u32)>>, i: usize, j: usize, color: &Vector3<f64>) {
    arr[i][j].0 += color.x;
    arr[i][j].1 += color.y;
    arr[i][j].2 += color.z;
    arr[i][j].3 += 1;
}

#[allow(dead_code)]
pub fn thread_safe_increment_color(arr: &Arc<Mutex<Vec<Vec<(f64, f64, f64, u32)>>>>, i: usize, j: usize, color: &Vector3<f64>) {
    let mut data = arr.lock().unwrap();
    data[i][j].0 += color.x;
    data[i][j].1 += color.y;
    data[i][j].2 += color.z;
    data[i][j].3 += 1;
}

pub fn thread_safe_update_image(arr: &Arc<Mutex<Vec<Vec<(f64, f64, f64, u32)>>>>, local: &Vec<Vec<(f64, f64, f64, u32)>>) {
    let mut data = arr.lock().unwrap();
    for i in 0usize..IMAGE_HEIGHT as usize {
        for j in 0usize..IMAGE_WIDTH as usize {
            let (r, g, b, n) = local[i][j];
            data[i][j].0 += r;
            data[i][j].1 += g;
            data[i][j].2 += b;
            data[i][j].3 += n;
        }
    }
}

pub fn refract(vec: &Unit<Vector3<f64>>, n: &Unit<Vector3<f64>>, eta: f64) -> Vector3<f64> {
    let cost = -vec.dot(n);
    let out_para = (vec.as_ref() + n.as_ref().scale(cost)).scale(eta);
    let out_perp = -n.scale((1. - out_para.dot(&out_para)).sqrt());
    out_para + out_perp
}

#[allow(dead_code)]
pub fn draw_picture(image: &mut RgbImage, pixels: &Vec<Vec<(f64, f64, f64, u32)>>, path: &str) {
    for i in 0..image.height() {
        let w = i as usize;
        for j in 0..image.width() {
            let (r, g, b, n) = pixels[w][j as usize];
            let pt = Point3::new(r, g, b);
            let color = point_to_color(&pt, 1. / GAMMA, n);
            image.put_pixel(j, i, color);
        }
    }
    image.save(path).unwrap();
}

fn point_to_color(vec: &Point3<f64>, gamma: f64, samples: u32) -> Rgb<u8> {
    let scale: f64 = 1. / (samples as f64);
    let r: f64 = clamp(vec[0] * scale, 0., 255.) * INV_COL_MAX;
    let g: f64 = clamp(vec[1] * scale, 0., 255.) * INV_COL_MAX;
    let b: f64 = clamp(vec[2] * scale, 0., 255.) * INV_COL_MAX;
    Rgb([(r.powf(gamma) * 256.).round() as u8,
         (g.powf(gamma) * 256.).round() as u8,
         (b.powf(gamma) * 256.).round() as u8])

}

pub fn schlick(cosine: f64, index: f64) -> f64 {
    let r0 = (1. - index) / (1. + index);
    let r0 = r0 * r0;
    return r0 + (1. - r0) * (1. - cosine).powf(5.);
}

pub fn thread_safe_draw_picture(img: &Mutex<image::RgbImage>, pixels: &Mutex<Vec<Vec<(f64, f64, f64, u32)>>>, path: &str) {
    let mut img_guard = img.lock().unwrap();
    let pixels_guard = pixels.lock().unwrap();

    for i in 0..img_guard.height() {
        let w = i as usize;
        for j in 0..img_guard.width() {
            let (r, g, b, n) = pixels_guard[w][j as usize];
            let pt = Point3::new(r / n as f64, g / n as f64, b / n as f64);
            let color = point_to_color(&pt, 1. / GAMMA, 1);
            img_guard.put_pixel(j, i, color);
        }
    }
    img_guard.save(path).unwrap();
}

#[allow(dead_code)]
pub fn get_sky(ray: &geometry::Ray) -> Vector3<f64> {
    let white = Rgb([255u8, 255u8, 255u8]);
    let blue = Rgb([80u8, 159u8, 255u8]);
    let unit: Unit<Vector3<f64>> = Unit::new_normalize(ray.dir);
    let color = gradient(&white, &blue, 0.5 * (1.0 + unit.as_ref().y));
    return Vector3::new(color[0] as f64, color[1] as f64, color[2] as f64);
}

pub fn get_background(_ray: &geometry::Ray) -> Vector3<f64> {Vector3::new(0., 0., 0.)}

pub fn get_sphere_uv(p: Vector3<f64>, record: &mut hittable::HitRecord) {
    let phi = p.z.atan2(p.x);
    let theta = p.y.asin();
    let u = 1. - (phi + PI) / (2. * PI);
    let v = (theta + PI / 2.) / PI;
    record.uv = Vector2::new(u, v);
}

pub fn get_new_box(bbox: hittable::BoundingBox, t: &Arc<Projective3<f64>>) -> hittable::BoundingBox {
    let mut min: Point3<f64> = Point3::new(INFINITY, INFINITY, INFINITY);
    let mut max: Point3<f64> = Point3::new(-INFINITY, -INFINITY,  -INFINITY);
    for i in 0..2 {
        for j in 0..2 {
            for k in 0..2 {
                let x = if i == 0 { bbox.min.x } else { bbox.max.x };
                let y = if j == 0 { bbox.min.y } else { bbox.max.y };
                let z = if k == 0 { bbox.min.z } else { bbox.max.z };
                let p = Point3::new(x, y , z);
                let np = t.transform_point(&p);
                min.x = min.x.min(np.x);
                min.y = min.y.min(np.y);
                min.z = min.z.min(np.z);
                max.x = max.x.max(np.x);
                max.y = max.y.max(np.y);
                max.z = max.z.max(np.z);
            }
        }
    }
    hittable::BoundingBox::new(min, max)
}

pub fn make_empty_image() -> Vec<Vec<(f64, f64, f64, u32)>> {
    let mut pixels: Vec<Vec<(f64, f64, f64, u32)>> = Vec::new();
    for _ in 0..IMAGE_HEIGHT {
        let mut temp: Vec<(f64, f64, f64, u32)> = Vec::new();
        for _ in 0..IMAGE_WIDTH {
            temp.push((0., 0., 0., 0u32));
        }
        pixels.push(temp);
    }
    pixels
}

#[allow(dead_code)]
fn box_compare(a: &Primitive, b: &Primitive, axis: usize) -> std::cmp::Ordering {
    let box_a = Primitive::get_bounding_box(a, 0., 0.);
    let box_b = Primitive::get_bounding_box(b, 0., 0.);
    if box_a.is_none() || box_b.is_none() {
        println!("Error, cannot compare objects");
        return std::cmp::Ordering::Equal;
    }
    match box_a {
        Some(bound_a) => {
            match box_b {
                Some(bound_b) => {
                    return if bound_a.min[axis] < bound_b.min[axis] { std::cmp::Ordering::Less } else { std::cmp::Ordering::Greater }
                }
                None => {panic!("Code in box_compare should not be reached: 1")}
            }
        }
        None => {panic!("Code in box_compare should not be reached: 2")}
    }
}

#[allow(dead_code)]
pub fn box_x_compare(a: &Primitive, b: &Primitive) -> std::cmp::Ordering { box_compare(a, b, 0) }
#[allow(dead_code)]
pub fn box_y_compare(a: &Primitive, b: &Primitive) -> std::cmp::Ordering { box_compare(a, b, 1) }
#[allow(dead_code)]
pub fn box_z_compare(a: &Primitive, b: &Primitive) -> std::cmp::Ordering { box_compare(a, b, 2) }