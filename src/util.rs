use image::{Rgb, RgbImage};
use nalgebra::geometry::Point3;
use nalgebra::base::{Unit, Vector3};
use rand::prelude::*;
use rand::distributions::Standard;

use crate::consts::*;

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

pub fn rand_in_hemisphere(normal: &Vector3<f64>) -> Vector3<f64> {
    let vec: Vector3<f64> = rand_in_unit_sphere();
    if normal.dot(&vec) > 0. {
        vec
    } else {
        -vec
    }
}

pub fn multiply_vector3(a: Vector3<f64>, b: Vector3<f64>) -> Vector3<f64> {
    let factor = 1. / 255.;
    Vector3::new(a.x * b.x * factor, a.y * b.y * factor, a.z * b.z * factor)
}

pub fn reflect(v: &Vector3<f64>, n: &Unit<Vector3<f64>>) -> Vector3<f64> {
    let scale = 2. * v.dot(n);
    v - n.as_ref().scale(scale)
}

#[allow(dead_code)]
pub fn increment_color(arr: &mut Vec<Vec<(f64, f64, f64)>>, i: usize, j: usize, color: &Vector3<f64>, samples: u32) {
    let inv = 1. / (samples as f64);
    let r = color.x * inv;
    let g = color.y * inv;
    let b = color.z * inv;
    arr[i][j].0 += r;
    arr[i][j].1 += g;
    arr[i][j].2 += b;
}

pub fn refract(vec: &Unit<Vector3<f64>>, n: &Unit<Vector3<f64>>, eta: f64) -> Vector3<f64> {
    let cost = -vec.dot(n);
    let out_para = (vec.as_ref() + n.as_ref().scale(cost)).scale(eta);
    let out_perp = -n.scale((1. - out_para.dot(&out_para)).sqrt());
    out_para + out_perp
}

#[allow(dead_code)]
pub fn draw_picture(image: &mut RgbImage, pixels: &Vec<Vec<(f64, f64, f64)>>, path: &str) {
    for i in 0..image.height() {
        let w = i as usize;
        for j in 0..image.width() {
            let (r, g, b) = pixels[w][j as usize];
            let pt = Point3::new(r, g, b);
            let color = point_to_color(&pt, 1. / GAMMA, 1);
            image.put_pixel(j, i, color);
        }
    }
    image.save(path).unwrap();
}

fn point_to_color(vec: &Point3<f64>, gamma: f64, samples: u32) -> Rgb<u8> {
    let scale: f64 = 1. / (255. * samples as f64);
    let r: f64 = vec[0] * scale;
    let g: f64 = vec[1] * scale;
    let b: f64 = vec[2] * scale;
    Rgb([(r.powf(gamma) * 255.).round() as u8,
         (g.powf(gamma) * 255.).round() as u8,
         (b.powf(gamma) * 255.).round() as u8])

}

pub fn schlick(cosine: f64, index: f64) -> f64 {
    let r0 = (1. - index) / (1. + index);
    let r0 = r0 * r0;
    return r0 + (1. - r0) * (1. - cosine).powf(5.);
}