use hittable::HitRecord;
use image::{Rgb, RgbImage};
use nalgebra::base::{Unit, Vector2, Vector3};
use nalgebra::{
    geometry::{Point2, Point3, Projective3},
    Matrix3,
};
use rand::distributions::{Standard, Uniform};
use rand::prelude::*;
use std::{
    collections::HashSet,
    sync::{Arc, Mutex},
};

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
    x.max(min).min(max)
}

pub fn rand() -> f64 {
    StdRng::from_entropy().sample(Uniform::new(0f64, 1f64))
}

pub fn rand_range(min: f64, max: f64) -> f64 {
    return min + (max - min) * rand();
}

pub fn uniform_sample_cone(u: &Point2<f64>, cos_theta_max: f64) -> Vector3<f64> {
    let cos_theta = (1f64 - u.x) + u.x * cos_theta_max;
    let sin_theta = (1f64 - cos_theta * cos_theta).sqrt();
    let phi = u.y * 2f64 * PI;
    Vector3::new(phi.cos() * sin_theta, phi.sin() * sin_theta, cos_theta)
}

pub fn uniform_cone_pdf(cos_theta_max: f64) -> f64 {
    1f64 / (2f64 * PI * (1f64 - cos_theta_max))
}

pub fn uniform_sample_sphere(u: &Point2<f64>) -> Vector3<f64> {
    let z: f64 = 1f64 - 2f64 * u[0];
    let r = 0f64.max(1f64 - z * z).sqrt();
    let phi: f64 = 2f64 * PI * u[1];
    Vector3::new(r * phi.cos(), r * phi.sin(), z)
}

pub fn uniform_sphere_pdf() -> f64 {
    INV_PI / 4f64
}

pub fn uniform_sample_triangle(u: &Point2<f64>) -> Point2<f64> {
    let s0 = u[0].sqrt();
    Point2::new(1f64 - s0, u[1] * s0)
}

// uniform sample triangle pdf is 1/area

pub fn cosine_sample_hemisphere(u: &Point2<f64>) -> Vector3<f64> {
    let d = concentric_sample_disk(u);
    let z = 0f64.max(1f64 - d.x * d.x - d.y * d.y).sqrt();
    Vector3::new(d.x, d.y, z)
}

pub fn cosine_hemisphere_pdf(cos_theta: f64) -> f64 {
    cos_theta * INV_PI
}

pub fn concentric_sample_disk(u: &Point2<f64>) -> Point2<f64> {
    let u_offset = 2f64 * u - Vector2::new(1f64, 1f64);
    if u_offset.x == 0f64 && u_offset.y == 0f64 {
        return Point2::new(0., 0.);
    }
    let theta: f64;
    let r: f64;
    if u_offset.x.abs() > u_offset.y.abs() {
        r = u_offset.x;
        theta = PI / 4f64 * (u_offset.y / u_offset.x);
    } else {
        r = u_offset.y;
        theta = PI / 2f64 - PI / 4f64 * (u_offset.x / u_offset.y);
    }
    Point2::new(r * theta.cos(), r * theta.sin())
}

#[allow(dead_code)]
pub fn rand_in_hemisphere(normal: &Vector3<f64>) -> Vector3<f64> {
    let vec: Vector3<f64> = uniform_sample_sphere(&Point2::new(rand(), rand()));
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
        if x * x + y * y < 1. {
            return Vector3::new(x, y, 0.);
        }
    }
}

pub fn rand_int(min: i32, max: i32) -> i32 {
    rand_range(min as f64, max as f64).round() as i32
}

pub fn rand_vector(min: f64, max: f64) -> Vector3<f64> {
    Vector3::new(
        rand_range(min, max),
        rand_range(min, max),
        rand_range(min, max),
    )
}

pub fn rand_cosine_dir() -> Vector3<f64> {
    let r1 = rand();
    let r2 = rand();
    let u1 = 2. * r1 - 1.;
    let u2 = 2. * r2 - 1.;
    if u1 == 0. && u2 == 0. {
        return Vector3::new(0., 0., 1.);
    }
    let theta: f64;
    let r: f64;
    if u1.abs() > u2.abs() {
        r = u1;
        theta = PI / 4. * (u2 / u1);
    } else {
        r = u2;
        theta = PI / 2. - PI / 4. * (u1 / u2);
    }
    let x = r * theta.cos();
    let y = r * theta.sin();
    let z = (0f64.max(1. - x * x - y * y)).sqrt();
    Vector3::new(x, y, z)
}

/**
Assumes normalized vector
*/
pub fn spherical_theta(v: &Vector3<f64>) -> f64 {
    clamp(v.y, -1f64, 1f64).acos()
}

/**
Assumes normalized vector
*/
pub fn spherical_phi(v: &Vector3<f64>) -> f64 {
    let p: f64 = v.z.atan2(v.x);
    if p < 0f64 {
        p + 2f64 * PI
    } else {
        p
    }
}

pub fn color_to_luminance(v: &Vector3<f64>) -> f64 {
    0.2126 * v.x + 0.7152 * v.y + 0.0722 * v.z
}

pub fn lerp(t: f64, min: f64, max: f64) -> f64 {
    (1f64 - t) * min + t * max
}

pub fn lerp_v(t: f64, min: &Vector3<f64>, max: &Vector3<f64>) -> Vector3<f64> {
    Vector3::new(
        lerp(t, min.x, max.x),
        lerp(t, min.y, max.y),
        lerp(t, min.z, max.z),
    )
}

pub fn hable(x: f64) -> f64 {
    let a = 0.15;
    let b = 0.5;
    let c = 0.1;
    let d = 0.2;
    let e = 0.02;
    let f = 0.3;
    ((x * (a * x + c * b) + d * e) / (x * (a * x + b) + d * f)) - e / f
}

pub fn permute_vec(vec: &Vector3<f64>, x: usize, y: usize, z: usize) -> Vector3<f64> {
    Vector3::new(vec[x], vec[y], vec[z])
}

pub fn permute_pt(vec: &Point3<f64>, x: usize, y: usize, z: usize) -> Point3<f64> {
    Point3::new(vec[x], vec[y], vec[z])
}

pub fn reflect(v: &Vector3<f64>, n: &Unit<Vector3<f64>>) -> Vector3<f64> {
    let scale = 2. * v.dot(n);
    -v + n.as_ref().scale(scale)
}

pub fn increment_color(
    arr: &mut Vec<Vec<(f64, f64, f64, u32)>>,
    i: usize,
    j: usize,
    color: &Vector3<f64>,
) {
    let height = arr.len();
    let width = arr[0].len();
    arr[i % height][j % width].0 += if color.x == std::f64::NAN {
        0.
    } else {
        color.x
    };
    arr[i % height][j % width].1 += if color.y == std::f64::NAN {
        0.
    } else {
        color.y
    };
    arr[i % height][j % width].2 += if color.z == std::f64::NAN {
        0.
    } else {
        color.z
    };
    arr[i % height][j % width].3 += 1;
}

#[allow(dead_code)]
pub fn thread_safe_write_pixel(
    arr: &Arc<Mutex<Vec<Vec<(f64, f64, f64, u32)>>>>,
    i: usize,
    j: usize,
    color: (f64, f64, f64, u32),
) {
    let mut data = arr.lock().unwrap();
    data[i][j].0 += if color.0 == std::f64::NAN {
        0.
    } else {
        color.0
    };
    data[i][j].1 += if color.1 == std::f64::NAN {
        0.
    } else {
        color.1
    };
    data[i][j].2 += if color.2 == std::f64::NAN {
        0.
    } else {
        color.2
    };
    data[i][j].3 += color.3;
}

#[allow(dead_code)]
pub fn thread_safe_increment_color(
    arr: &Arc<Mutex<Vec<Vec<(f64, f64, f64, u32)>>>>,
    i: usize,
    j: usize,
    color: &Vector3<f64>,
) {
    let mut data = arr.lock().unwrap();
    data[i][j].0 += if color.x == std::f64::NAN {
        0.
    } else {
        color.x
    };
    data[i][j].1 += if color.y == std::f64::NAN {
        0.
    } else {
        color.y
    };
    data[i][j].2 += if color.z == std::f64::NAN {
        0.
    } else {
        color.z
    };
    data[i][j].3 += 1;
}

pub fn thread_safe_update_image(
    arr: &Arc<Mutex<Vec<Vec<(f64, f64, f64, u32)>>>>,
    local: &Vec<Vec<(f64, f64, f64, u32)>>,
) {
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

pub fn thread_safe_draw_picture(
    img: &Mutex<image::RgbImage>,
    pixels: &Mutex<Vec<Vec<(f64, f64, f64, u32)>>>,
    changed_tiles: &HashSet<(usize, usize)>,
    path: &str,
) {
    let mut img_guard = img.lock().unwrap();
    let pixels_guard = pixels.lock().unwrap();

    for i in 0..img_guard.height() {
        let w = i as usize;
        for j in 0..img_guard.width() {
            let tile_x: usize = (j / TILE_SIZE) as usize;
            let tile_y: usize = (i / TILE_SIZE) as usize;
            let (r, g, b, n) = pixels_guard[w][j as usize];
            let pt = Point3::new(r / n as f64, g / n as f64, b / n as f64);
            let mut color = point_to_color(&pt, 1. / GAMMA, 1);
            if changed_tiles.contains(&(tile_x, tile_y)) {
                let mod_x: usize = (j % TILE_SIZE) as usize;
                let mod_y: usize = (i % TILE_SIZE) as usize;
                // yellow border around active tiles
                if (mod_x == 0
                    && (mod_y < TILE_SIZE as usize / 4 || mod_y > 3 * TILE_SIZE as usize / 4))
                    || (mod_x == TILE_SIZE as usize - 1
                        && (mod_y < TILE_SIZE as usize / 4 || mod_y > 3 * TILE_SIZE as usize / 4))
                    || (mod_y == 0
                        && (mod_x < TILE_SIZE as usize / 4 || mod_x > 3 * TILE_SIZE as usize / 4))
                    || (mod_y == TILE_SIZE as usize - 1
                        && (mod_x < TILE_SIZE as usize / 4 || mod_x > 3 * TILE_SIZE as usize / 4))
                {
                    color = Rgb([255, 211, 0]);
                }
            }
            img_guard.put_pixel(j, i, color);
        }
    }
    let res = img_guard.save(path);
    match res {
        Ok(_) => {}
        Err(_) => {
            std::thread::sleep(std::time::Duration::from_secs(1)); // just wait and try again
            img_guard.save(path).unwrap();
        }
    }
}

pub fn progressive_draw_picture(
    img: &Mutex<image::RgbImage>,
    pixels: &Mutex<Vec<Vec<(f64, f64, f64, u32)>>>,
    path: &str,
) {
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
    let res = img_guard.save(path);
    match res {
        Ok(_) => {}
        Err(_) => {
            std::thread::sleep(std::time::Duration::from_secs(1)); // just wait and try again
            img_guard.save(path).unwrap();
        }
    }
}

/**
* Refract vec over `n` with given IoR
*/
pub fn refract(vec: &Vector3<f64>, n: &Unit<Vector3<f64>>, eta: f64) -> Option<Vector3<f64>> {
    let cos_theta_i = n.dot(vec) / vec.magnitude(); // make sure cos_theta_i is in [0, 1]
    let sin2_theta_i = 0f64.max(1f64 - cos_theta_i * cos_theta_i);
    let sin2_theta_t = eta * eta * sin2_theta_i;
    if sin2_theta_t >= 1. {
        return None;
    }
    let cos_theta_t = (1f64 - sin2_theta_t).sqrt();
    return Some(eta * -vec + (eta * cos_theta_i - cos_theta_t) * Vector3::new(n.x, n.y, n.z));
}

#[allow(dead_code)]
pub fn draw_picture(image: &mut RgbImage, pixels: &Vec<Vec<(f64, f64, f64, u32)>>, path: &String) {
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

fn aces_comp(x: f64) -> f64 {
    let x = x * 0.6;
    let a = 2.51;
    let b = 0.03;
    let c = 2.43;
    let d = 0.59;
    let e = 0.14;
    return clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0f64, 1f64);
}

fn aces_computation_component(x: f64) -> f64 {
    let a = x * (x + 0.0245786) - 0.000090537;
    let b = x * (0.983729 * x + 0.4329510) + 0.238081;
    a / b
}

fn aces_computation(v: &Vector3<f64>) -> Vector3<f64> {
    Vector3::new(
        aces_computation_component(v.x),
        aces_computation_component(v.y),
        aces_computation_component(v.z),
    )
}

fn aces_full(v: &Vector3<f64>) -> Vector3<f64> {
    let input = Matrix3::new(
        0.59719, 0.35458, 0.04823, 0.076, 0.9083, 0.01566, 0.0284, 0.13383, 0.833777,
    );
    let output = Matrix3::new(
        1.60475, -0.53108, -0.07367, -0.10208, 1.10813, -0.00605, -0.00327, -0.07276, 1.0762,
    );

    let v = input * v;
    let v = aces_computation(&v);
    output * v
}

fn aces_approx(v: &Vector3<f64>) -> Vector3<f64> {
    Vector3::new(aces_comp(v.x), aces_comp(v.y), aces_comp(v.z))
}

fn point_to_color(vec: &Point3<f64>, gamma: f64, samples: u32) -> Rgb<u8> {
    let scale: f64 = 1. / (samples as f64);
    let mut r: f64 = vec[0] * scale;
    let mut g: f64 = vec[1] * scale;
    let mut b: f64 = vec[2] * scale;
    // apply tonemapping
    if TONE_MAPPING {
        // let sig = r.max(g).max(b);
        let new_color = Vector3::new(r, g, b);
        // // second vector comes from eye response
        // let luma = color_to_luminance(&new_color);
        // let coeff = (sig - 0.18).max(1e-6) / sig.max(1e-6);
        // let coeff = coeff.powi(20);
        // let new_color = mix(&new_color, &Vector3::new(luma, luma, luma), coeff);
        // let sig = mix_f(sig, luma, coeff);
        // let final_color = new_color.scale(hable(sig) / sig);
        let final_color = aces_approx(&new_color);
        r = final_color[0];
        g = final_color[1];
        b = final_color[2];
    } else {
        r = clamp(r, 0f64, 1f64);
        g = clamp(g, 0f64, 1f64);
        b = clamp(b, 0f64, 1f64);
    }
    Rgb([
        (r.powf(gamma) * 256.).round() as u8,
        (g.powf(gamma) * 256.).round() as u8,
        (b.powf(gamma) * 256.).round() as u8,
    ])
}

pub fn mix(from: &Vector3<f64>, to: &Vector3<f64>, x: f64) -> Vector3<f64> {
    from.scale(1f64 - x) + to.scale(x)
}

pub fn mix_f(from: f64, to: f64, x: f64) -> f64 {
    from * (1f64 - x) + to * x
}

#[allow(dead_code)]
pub fn get_sky(ray: &geometry::Ray) -> Vector3<f64> {
    let white = Rgb([255u8, 255u8, 255u8]);
    let blue = Rgb([140u8, 159u8, 185u8]);
    let unit: Unit<Vector3<f64>> = Unit::new_normalize(ray.dir);
    let color = gradient(&white, &blue, 0.5 * (1.0 + unit.as_ref().y));
    return Vector3::new(
        color[0] as f64 * INV_COL_MAX,
        color[1] as f64 * INV_COL_MAX,
        color[2] as f64 * INV_COL_MAX,
    );
}

pub fn get_background(_ray: &geometry::Ray) -> Vector3<f64> {
    Vector3::new(0., 0., 0.)
}

pub fn get_new_box(
    bbox: hittable::BoundingBox,
    t: &Arc<Projective3<f64>>,
) -> hittable::BoundingBox {
    let mut min: Point3<f64> = Point3::new(INFINITY, INFINITY, INFINITY);
    let mut max: Point3<f64> = Point3::new(-INFINITY, -INFINITY, -INFINITY);
    for i in 0..2 {
        for j in 0..2 {
            for k in 0..2 {
                let x = if i == 0 { bbox.min.x } else { bbox.max.x };
                let y = if j == 0 { bbox.min.y } else { bbox.max.y };
                let z = if k == 0 { bbox.min.z } else { bbox.max.z };
                let p = Point3::new(x, y, z);
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

pub fn make_empty_image(height: usize, width: usize) -> Vec<Vec<(f64, f64, f64, u32)>> {
    let mut pixels: Vec<Vec<(f64, f64, f64, u32)>> = Vec::new();
    for _ in 0..height {
        let mut temp: Vec<(f64, f64, f64, u32)> = Vec::new();
        for _ in 0..width {
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
        Some(bound_a) => match box_b {
            Some(bound_b) => {
                return if bound_a.min[axis] < bound_b.min[axis] {
                    std::cmp::Ordering::Less
                } else {
                    std::cmp::Ordering::Greater
                }
            }
            None => {
                panic!("Code in box_compare should not be reached: 1")
            }
        },
        None => {
            panic!("Code in box_compare should not be reached: 2")
        }
    }
}

#[allow(dead_code)]
pub fn box_x_compare(a: &Primitive, b: &Primitive) -> std::cmp::Ordering {
    box_compare(a, b, 0)
}
#[allow(dead_code)]
pub fn box_y_compare(a: &Primitive, b: &Primitive) -> std::cmp::Ordering {
    box_compare(a, b, 1)
}
#[allow(dead_code)]
pub fn box_z_compare(a: &Primitive, b: &Primitive) -> std::cmp::Ordering {
    box_compare(a, b, 2)
}

pub fn make_coordinate_system(v1: &Vector3<f64>) -> (Vector3<f64>, Vector3<f64>) {
    let v2: Vector3<f64>;
    if v1.x.abs() > v1.y.abs() {
        v2 = Vector3::new(-v1.z, 0., v1.x) * (1. / (v1.x * v1.x + v1.z * v1.z).sqrt());
    } else {
        v2 = Vector3::new(0., v1.z, -v1.y) * (1. / (v1.y * v1.y + v1.z * v1.z).sqrt());
    }
    let v3 = v1.cross(&v2);
    (v2, v3)
}

pub fn face_forward(n: Unit<Vector3<f64>>, v: &Vector3<f64>) -> Unit<Vector3<f64>> {
    let d = n.dot(&v);
    return if d < 0. { -n } else { n };
}

pub fn white() -> Vector3<f64> {
    Vector3::new(1., 1., 1.)
}

pub fn black() -> Vector3<f64> {
    Vector3::new(0., 0., 0.)
}

pub fn same_hemisphere(v: &Vector3<f64>, w: &Vector3<f64>) -> bool {
    v.z * w.z > 0.
}

pub fn make_spherical(sin_t: f64, cos_t: f64, phi: f64) -> Vector3<f64> {
    Vector3::new(sin_t * phi.cos(), sin_t * phi.sin(), cos_t)
}

pub fn correct_shading_normal(
    record: &HitRecord,
    wo: &Vector3<f64>,
    wi: &Vector3<f64>,
    mode: u8,
) -> f64 {
    if mode == RADIANCE {
        1f64
    } else {
        (wo.dot(&record.shading.n).abs() * wi.dot(&record.shading.n).abs())
            / (wo.dot(&record.n).abs() * wi.dot(&record.n).abs())
    }
}

// approximation of error function
pub fn erf(x: f64) -> f64 {
    let a1 = 0.254829592;
    let a2 = -0.284496736;
    let a3 = 1.421413741;
    let a4 = -1.453152027;
    let a5 = 1.061405429;
    let p = 0.3275911;

    let mut sign = 1;
    if x < 0. {
        sign *= -1;
    }
    let x = x.abs();
    let t = 1. / (1. + p * x);
    let y = 1. - (((((a5 * t + a4) * t) + a3) * t + a2) * t + a1) * t * (-x * x).exp();
    if sign == 1 {
        y
    } else {
        -y
    }
}

pub fn inv_erf(x: f64) -> f64 {
    let x = clamp(x, -0.99999, 0.99999);
    let mut w = -((1. - x) * (1. + x)).ln();
    let mut p: f64;
    if w < 5. {
        w = w - 2.5;
        p = 2.81022636e-08;
        p = 3.43273939e-07 + p * w;
        p = -3.5233877e-06 + p * w;
        p = -4.39150654e-06 + p * w;
        p = 0.00021858087 + p * w;
        p = -0.00125372503 + p * w;
        p = -0.00417768164 + p * w;
        p = 0.246640727 + p * w;
        p = 1.50140941 + p * w;
    } else {
        w = w.sqrt() - 3.;
        p = -0.000200214257;
        p = 0.000100950558 + p * w;
        p = 0.00134934322 + p * w;
        p = -0.00367342844 + p * w;
        p = 0.00573950773 + p * w;
        p = -0.0076224613 + p * w;
        p = 0.00943887047 + p * w;
        p = 1.00167406 + p * w;
        p = 2.83297682 + p * w;
    }
    p * x
}
