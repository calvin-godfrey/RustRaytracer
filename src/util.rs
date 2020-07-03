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
    let r = color.x;
    let g = color.y;
    let b = color.z;
    let scale = 1. / (samples as f64 * 255.);
    let r = r * scale;
    let g = g * scale;
    let b = b * scale;
    let ans = Rgb([(r.sqrt() * 255.).round() as u8,
                            (g.sqrt() * 255.).round() as u8,
                            (b.sqrt() * 255.).round() as u8]);
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

pub fn increment_color(img: &mut RgbImage, i: u32, j: u32, color: &Vector3<f64>, samples: u32) {
    let inv: f64 = 1. / (samples as f64);
    let curr = img.get_pixel(i, j);
    let r = curr[0] as f64 + color.x * inv;
    let g = curr[1] as f64 + color.y * inv;
    let b = curr[2] as f64 + color.z * inv;
    img.put_pixel(i, j, Rgb([r.round() as u8, g.round() as u8, b.round() as u8]));
}

pub fn refract(vec: &Unit<Vector3<f64>>, n: &Unit<Vector3<f64>>, eta: f64) -> Vector3<f64> {
    let cost = -vec.dot(n);
    let out_para = (vec.as_ref() + n.as_ref().scale(cost)).scale(eta);
    let out_perp = -n.scale((1. - out_para.dot(&out_para)).sqrt());
    out_para + out_perp
}