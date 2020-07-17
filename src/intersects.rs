use nalgebra::base::{Vector3, Vector2, Unit};
use nalgebra::geometry::{Projective3, Point3};
use std::sync::Arc;
use crate::geometry::Ray;
use crate::hittable::HitRecord;
use crate::util;
use crate::primitive::moving_sphere_center;

#[allow(unused_variables)]
pub fn xy_rect_intersect(x0: f64, y0: f64, x1: f64, y1: f64, k: f64, mat_index: usize, transform: &Option<Arc<Projective3<f64>>>, ray: &Ray, t0: f64, t1: f64) -> Option<HitRecord> {
    let transformed_ray: Ray = if transform.is_none() { *ray } else { ray.transform(transform.as_ref().unwrap()) };
    let dir = transformed_ray.dir;
    let origin = transformed_ray.origin;
    let time = transformed_ray.time;
    let t = (k - origin.z) / dir.z;
    if t < t0 || t > t1 {
        return None;
    }
    let x = origin.x + t * dir.x;
    let y = origin.y + t * dir.y;
    if x < x0 || y < y0 || x > x1 || y > y1 {
        return None;
    }
    let uv = Vector2::new((x-x0)/(x1-x0), (y-y0)/(y1-y0));
    let mut n = Vector3::new(0., 0., 1.);
    let mut p = transformed_ray.at(t);
    if transform.is_some() {
        n = transform.as_ref().unwrap().transform_vector(&n);
        p = transform.as_ref().unwrap().transform_point(&p);
    }
    let mut record = HitRecord::new(t, Unit::new_normalize(n), p, true, mat_index);
    record.set_front(&transformed_ray);
    record.uv = uv;
    Some(record)
}

#[allow(unused_variables)]
pub fn xz_rect_intersect(x0: f64, z0: f64, x1: f64, z1: f64, k: f64, mat_index: usize, transform: &Option<Arc<Projective3<f64>>>, ray: &Ray, t0: f64, t1: f64) -> Option<HitRecord> {
    let transformed_ray: Ray = if transform.is_none() { *ray } else { ray.transform(transform.as_ref().unwrap()) };
    let dir = transformed_ray.dir;
    let origin = transformed_ray.origin;
    let time = transformed_ray.time;
    let t = (k - origin.y) / dir.y;
    if t < t0 || t > t1 {
        return None;
    }
    let x = origin.x + t * dir.x;
    let z = origin.z + t * dir.z;
    if x < x0 || z < z0 || x > x1 || z > z1 {
        return None;
    }
    let uv = Vector2::new((x-x0)/(x1-x0), (z-z0)/(z1-z0));
    let mut n = Vector3::new(0., 1., 0.);
    let mut p = transformed_ray.at(t);
    if transform.is_some() {
        n = transform.as_ref().unwrap().transform_vector(&n);
        p = transform.as_ref().unwrap().transform_point(&p);
    }
    let mut record = HitRecord::new(t, Unit::new_normalize(n), p, true, mat_index);
    record.set_front(&transformed_ray);
    record.uv = uv;
    Some(record)
}

#[allow(unused_variables)]
pub fn yz_rect_intersect(y0: f64, z0: f64, y1: f64, z1: f64, k: f64, mat_index: usize, transform: &Option<Arc<Projective3<f64>>>, ray: &Ray, t0: f64, t1: f64) -> Option<HitRecord> {
    let transformed_ray: Ray = if transform.is_none() { *ray } else { ray.transform(transform.as_ref().unwrap()) };
    let dir = transformed_ray.dir;
    let origin = transformed_ray.origin;
    let time = transformed_ray.time;
    let t = (k - origin.x) / dir.x;
    if t < t0 || t > t1 {
        return None;
    }
    let y = origin.y + t * dir.y;
    let z = origin.z + t * dir.z;
    if y < y0 || z < z0 || y > y1 || z > z1 {
        return None;
    }
    let uv = Vector2::new((y-y0)/(y1-y0), (z-z0)/(z1-z0));
    let mut n = Vector3::new(1., 0., 0.);
    let mut p = transformed_ray.at(t);
    if transform.is_some() {
        n = transform.as_ref().unwrap().transform_vector(&n);
        p = transform.as_ref().unwrap().transform_point(&p);
    }
    let mut record = HitRecord::new(t, Unit::new_normalize(n), p, true, mat_index);
    record.set_front(&transformed_ray);
    record.uv = uv;
    Some(record)
}

pub fn sphere_intersect(center: &Point3<f64>, r: &f64, mat_index: usize, ray: &Ray, tmin: f64, tmax: f64) -> Option<HitRecord> {
    let diff: Vector3<f64> = ray.origin - center;
    // get quadratic equation, calculate discriminant
    let a = ray.dir.dot(&ray.dir);
    let b = diff.dot(&ray.dir);
    let c = diff.dot(&diff) - r * r;
    let disc = b * b - a * c;
    if disc < 0.0 {
        return None;
    }
    let inv_a = 1.0 / a;
    let root = disc.sqrt();
    let ans = (-b - root) * inv_a; // try first solution to equation
    let mut hit_record: HitRecord;
    if ans < tmax && ans > tmin {
        let hit = ray.at(ans);
        hit_record = HitRecord::new(ans, Unit::new_normalize(hit - center), hit, true, mat_index);
        hit_record.set_front(ray);
        hit_record.front = true; // no backside to sphere
    } else {
        let ans = (-b + root) * inv_a;
        if ans < tmax && ans > tmin {
            let hit = ray.at(ans);
            let mut hit_record = HitRecord::new(ans, Unit::new_normalize(hit - center), hit, true, mat_index);
            hit_record.set_front(ray);
            hit_record.front = true; // no backside to sphere
            return Some(hit_record);
        } else {
            return None;
        }
    }
    util::get_sphere_uv((hit_record.p - center).scale( 1. / *r), &mut hit_record);
    Some(hit_record)
}

pub fn moving_sphere_intersect(r: f64, mat_index: usize, t0: f64, t1: f64, c0: &Point3<f64>, c1: &Point3<f64>, ray: &Ray, tmin: f64, tmax: f64) -> Option<HitRecord> {
    let center = moving_sphere_center(c0, c1, t0, t1, ray.time);
    let diff: Vector3<f64> = ray.origin - center;
    // get quadratic equation, calculate discriminant
    let a = ray.dir.dot(&ray.dir);
    let b = diff.dot(&ray.dir);
    let c = diff.dot(&diff) - r * r;
    let disc = b * b - a * c;
    if disc < 0.0 {
        return None;
    }
    let inv_a = 1.0 / a;
    let root = disc.sqrt();
    let ans = (-b - root) * inv_a; // try first solution to equation
    if ans < tmax && ans > tmin {
        let hit = ray.at(ans);
        let mut hit_record = HitRecord::new(ans, Unit::new_normalize(hit - center), hit, true, mat_index);
        hit_record.set_front(ray);
        hit_record.front = true; // no backside to sphere
        return Some(hit_record);
    }
    let ans = (-b + root) * inv_a;
    if ans < tmax && ans > tmin {
        let hit = ray.at(ans);
        let mut hit_record = HitRecord::new(ans, Unit::new_normalize(hit - center), hit, true, mat_index);
        hit_record.set_front(ray);
        hit_record.front = true; // no backside to sphere
        return Some(hit_record);
    } else {
        return None;
    }
}