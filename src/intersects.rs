use nalgebra::base::{Vector3, Vector2, Unit};
use nalgebra::geometry::{Projective3, Point3};
use std::sync::Arc;
use crate::geometry::Ray;
use crate::hittable::HitRecord;
use crate::util;
use crate::primitive::moving_sphere_center;
use crate::primitive::Primitive;
use crate::consts::*;

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
    } else {
        let ans = (-b + root) * inv_a;
        if ans < tmax && ans > tmin {
            let hit = ray.at(ans);
            let mut hit_record = HitRecord::new(ans, Unit::new_normalize(hit - center), hit, true, mat_index);
            hit_record.set_front(ray);
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
        return Some(hit_record);
    }
    let ans = (-b + root) * inv_a;
    if ans < tmax && ans > tmin {
        let hit = ray.at(ans);
        let mut hit_record = HitRecord::new(ans, Unit::new_normalize(hit - center), hit, true, mat_index);
        hit_record.set_front(ray);
        return Some(hit_record);
    } else {
        return None;
    }
}

pub fn medium_intersects(shape: &Primitive, inv_density: f64, mat_index: usize, ray: &Ray, tmin: f64, tmax: f64) -> Option<HitRecord> {
    let transform = Primitive::get_transform(shape);
    let transformed_ray = if transform.is_none() { *ray } else { ray.transform(transform.as_ref().unwrap()) };
    let first_record = Primitive::intersects_obj(shape, ray, -INFINITY, INFINITY);
    if first_record.is_none() {
        // println!("1");
        return None;
    }
    let mut first_record = first_record.unwrap();
    let second_record  = Primitive::intersects_obj(shape, ray, first_record.t + 100. * SMALL, INFINITY);
    if second_record.is_none() {
        // println!("2");
        return None; // should only happen if the shape is infinitely long
    }
    let mut second_record = second_record.unwrap();
    if first_record.t < tmin {
        first_record.t = tmin;
    }
    if second_record.t > tmax {
        second_record.t = tmax;
    }
    if first_record.t >= second_record.t {
        return None;
    }
    if first_record.t < 0. {
        first_record.t = 0.;
    }

    let ray_length: f64 = transformed_ray.dir.magnitude();
    let distance_inside = (second_record.t - first_record.t) * ray_length;
    let hit = inv_density * util::rand().ln();
    if hit > distance_inside {
        return None;
    }

    let t = first_record.t + hit / ray_length;
    let p = transformed_ray.at(t);
    Some(HitRecord::new(t, Unit::new_normalize(Vector3::new(1., 0., 0.)), p, true, mat_index)) // TODO: Proper values?
}