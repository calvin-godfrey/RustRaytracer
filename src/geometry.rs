use nalgebra::base::Vector3;
use nalgebra::geometry::Point3;

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

pub fn cast_ray(ray: &Ray, vec: &Vec<Box<dyn Hittable>>, depth: u32) -> Vector3<f64> {
    if depth <= 0 {
        return Vector3::new(0.0, 0.0, 0.0);
    }
    let records: Vec<HitRecord> = vec.iter().filter_map(|x| x.intersects(ray, SMALL, INFINITY)).collect();
    let hit_record = records.iter().min_by(|x, y| x.t.partial_cmp(&y.t).unwrap());
    
    match hit_record {
        Some(record) => {
            let pair = record.mat.scatter(ray, &record);
            match pair {
                Some((x, y)) => {
                    let col = cast_ray(&x, vec, depth - 1);
                    return Vector3::new(col.x * y.x * INV_COL_MAX, col.y * y.y * INV_COL_MAX, col.z * y.z * INV_COL_MAX);
                },
                None => Vector3::new(0.0, 0.0, 0.0)
            }
        }
        None => util::get_sky(ray)
    }    
}