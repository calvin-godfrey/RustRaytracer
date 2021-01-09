use nalgebra::base::{Vector3, Vector2, Unit};
use nalgebra::geometry::{Projective3, Point3};
use std::sync::Arc;
use crate::geometry::Ray;
use crate::hittable::HitRecord;
use crate::util;
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
    let mut p = transformed_ray.at(t);
    let mut dpdu = Vector3::new(1., 0., 0.);
    let mut dpdv = Vector3::new(0., 1., 0.);
    if transform.is_some() {
        let trans = transform.as_ref().unwrap();
        p = trans.transform_point(&p);
        dpdu = trans.transform_vector(&dpdu);
        dpdv = trans.transform_vector(&dpdv);
    }
    let mut record = HitRecord::new(p, uv, -ray.dir, dpdu, dpdv, util::black(), util::black(), t, 0, mat_index);
    record.set_front(&ray); // p and n were transformed to world space, so use the original ray
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
    let mut p = transformed_ray.at(t);
    let mut dpdu = Vector3::new(1., 0., 0.);
    let mut dpdv = Vector3::new(0., 0., 1.);
    if transform.is_some() {
        let trans = transform.as_ref().unwrap();
        p = trans.transform_point(&p);
        dpdu = trans.transform_vector(&dpdu);
        dpdv = trans.transform_vector(&dpdv);
    }
    let mut record = HitRecord::new(p, uv, -ray.dir, dpdu, dpdv, util::black(), util::black(), t, 0, mat_index);
    record.set_front(&ray);
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
    let mut p = transformed_ray.at(t);
    let mut dpdu = Vector3::new(0., 1., 0.);
    let mut dpdv = Vector3::new(0., 0., 1.);
    if transform.is_some() {
        let trans = transform.as_ref().unwrap();
        p = trans.transform_point(&p);
        dpdu = trans.transform_vector(&dpdu);
        dpdv = trans.transform_vector(&dpdv);
    }
    let mut record = HitRecord::new(p, uv, -ray.dir, dpdu, dpdv, util::black(), util::black(), t, 0, mat_index);
    record.set_front(&ray);
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
    let t: f64;
    if ans < tmax && ans > tmin {
        t = ans;
    } else {
        let ans = (-b + root) * inv_a;
        if ans < tmax && ans > tmin {
            t = ans;
        } else {
            return None;
        }
    }
    // makes it so that we can compute with the center of the sphere at 0, 0, 0
    let translated_ray = Ray::new_time(ray.origin - center.coords, ray.dir, ray.time);
    let mut record = make_sphere_record(t, *r, mat_index, &translated_ray);
    record.p += center.coords;
    Some(record)
}

#[allow(non_snake_case)]
fn make_sphere_record(t: f64, r: f64, mat_index: usize, ray: &Ray) -> HitRecord {
    let p = ray.at(t);
    let mut p: Point3<f64> = p * r / (p - util::black()).coords.magnitude(); // Slightly higher accuracy
    if p.x == 0. && p.y == 0. {
        p.x = 1e-5 * r; // so that atan2 exists
    }
    let mut phi = p.y.atan2(p.x);
    if phi < 0f64 {
        phi = phi + 2f64 * PI;
    }
    let phi_max = 2f64 * PI; // TODO: Generalize, allow partial spheres
    let theta_min = 0f64;
    let theta_max = PI;
    let u = phi / phi_max;
    let theta = util::clamp(p.z / r, -1., 1.).acos();
    let v = (theta - theta_min) / (theta_max - theta_min);
    let z_r = (p.x * p.x + p.y * p.y).sqrt();
    let inv_z_r = 1f64 / z_r;
    let cos_phi = p.x * inv_z_r;
    let sin_phi = p.y * inv_z_r;
    let dpdu = Vector3::new(-phi_max * p.y, phi_max * p.x, 0.);
    let dpdv = (theta_max - theta_min) * Vector3::new(p.z * cos_phi, p.z * sin_phi, -r * theta.sin());
    // big linear algebra
    let d2pduu = -phi_max * phi_max * Vector3::new(p.x, p.y, 0.);
    let d2pduv = (theta_max - theta_min) * p.z * phi_max * Vector3::new(-sin_phi, cos_phi, 0.);
    let d2pdvv = -(theta_max - theta_min) * (theta_max - theta_min) * p.coords;
    let E = dpdu.dot(&dpdu);
    let F = dpdu.dot(&dpdv);
    let G = dpdv.dot(&dpdv);
    let N = Unit::new_normalize(dpdu.cross(&dpdv));
    let e = N.dot(&d2pduu);
    let f = N.dot(&d2pduv);
    let g = N.dot(&d2pdvv);
    let inv_egf2= 1f64 / (E * G - F * F);
    let dndu = (f * F - e * G) * inv_egf2 * dpdu + (e * F - f * E) * inv_egf2 * dpdv;
    let dndv = (g * F - f * G) * inv_egf2 * dpdu + (f * F - g * E) * inv_egf2 * dpdv;
    // fill out hitrecord
    let uv = Vector2::new(u, v);
    let mut record = HitRecord::new(p, uv, -ray.dir, dpdu, dpdv, dndu, dndv, t, 0, mat_index);
    record.set_front(&ray);
    record
}
