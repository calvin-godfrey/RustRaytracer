use nalgebra::base::{Unit, Vector3};
use nalgebra::geometry::Point3;
use image::Rgb;

use crate::hittable::{Hittable, HitRecord};
use crate::consts::*;
use crate::util;
pub struct Ray {
    pub origin: Point3<f64>,
    pub dir: Vector3<f64>,
}

impl Ray {
    pub fn new(origin: Point3<f64>, dir: Vector3<f64>) -> Self { Self { origin, dir } }

    pub fn at(&self, t: f64) -> Point3<f64> {
        self.origin + self.dir.scale(t)
    }
}

pub fn cast_ray(ray: &Ray, vec: &Vec<Box<dyn Hittable>>, light: &Point3<f64>, depth: u32) -> Vector3<f64> {
    if depth <= 0 {
        return Vector3::new(0.0, 0.0, 0.0);
    }
    let mut hit_record: Option<HitRecord> = None;
    for sphere in vec.iter() {
        let attempt: Option<HitRecord> = sphere.intersects(ray, SMALL, INFINITY);
        match attempt {
            Some(x) => {
                match hit_record {
                    Some(y) => {
                        if x.t < y.t {
                            hit_record = Some(x);
                        }
                    }
                    None => { hit_record = Some(x) }
                }
            }
            None => {}
        }
    }
    match hit_record {
        Some(record) => {
            let pair = record.mat.scatter(ray, &record);
            match pair.0 {
                Some(x) => {
                    match pair.1 {
                        Some(y) => {
                            return util::multiply_vector3(y, cast_ray(&x, vec, light, depth - 1));
                        }
                        None => {return Vector3::new(0.0, 0.0, 0.0)} // should never happen
                    }
                }
                None => {return Vector3::new(0.0, 0.0, 0.0)}
            }
        }
        None => {
            let white = Rgb([255u8, 255u8, 255u8]);
            let blue = Rgb([50u8, 129u8, 255u8]);
            let unit: Unit<Vector3<f64>> = Unit::new_normalize(ray.dir);
            let color = util::gradient(&white, &blue, 0.5 * (1.0 + unit.as_ref().y));
            return Vector3::new(color[0] as f64, color[1] as f64, color[2] as f64);
        }
    }    
}