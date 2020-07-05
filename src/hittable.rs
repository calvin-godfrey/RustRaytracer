use nalgebra::base::{Unit, Vector3};
use nalgebra::geometry::Point3;
use crate::material::materials::Material;
use crate::geometry::Ray;

#[derive(Copy, Clone)]
pub struct HitRecord<'a> {
    pub t: f64, // time of hit along ray
    pub n: Unit<Vector3<f64>>, // normal of surface at point
    pub p: Point3<f64>, // point of intersection
    pub front: bool, // if the normal points outwards or not
    pub mat: &'a Box<dyn Material>, // how the surface acts
}

impl <'a> HitRecord<'a> {
    pub fn new(t: f64, n: Unit<Vector3<f64>>, p: Point3<f64>, front: bool, mat: &'a Box<dyn Material>) -> Self { Self { t, n, p, front, mat } }
    pub fn set_front(&mut self, ray: &Ray) {
        self.front = ray.dir.dot(self.n.as_ref()) < 0.0;
        self.n = if self.front { self.n } else { -self.n }
    }
}

pub trait Hittable: Send + Sync {
    fn intersects(&self, ray: &Ray, tmin: f64, tmax: f64) -> Option<HitRecord>;
}

pub struct Sphere {
    center: Point3<f64>,
    r: f64,
    mat: Box<dyn Material>,
}

impl Sphere {
    pub fn new(center: Point3<f64>, r: f64, mat: Box<dyn Material>) -> Self { Self { center, r, mat } }
}

impl Hittable for Sphere {
    fn intersects(&self, ray: &Ray, tmin: f64, tmax: f64) -> Option<HitRecord> {
        let diff: Vector3<f64> = ray.origin - self.center;
        // get quadratic equation, calculate discriminant
        let a = ray.dir.dot(&ray.dir);
        let b = diff.dot(&ray.dir);
        let c = diff.dot(&diff) - self.r * self.r;
        let disc = b * b - a * c;
        if disc < 0.0 {
            return None;
        }
        let inv_a = 1.0 / a;
        let root = disc.sqrt();
        let ans = (-b - root) * inv_a; // try first solution to equation
        if ans < tmax && ans > tmin {
            let hit = ray.at(ans);
            let mut hit_record = HitRecord::new(ans, Unit::new_normalize(hit - self.center), hit, true, &self.mat);
            hit_record.set_front(ray);
            return Some(hit_record);
        }
        let ans = (-b + root) * inv_a;
        if ans < tmax && ans > tmin {
            let hit = ray.at(ans);
            let mut hit_record = HitRecord::new(ans, Unit::new_normalize(hit - self.center), hit, true, &self.mat);
            hit_record.set_front(ray);
            return Some(hit_record);
        } else {
            return None;
        }
    }
}

// to please compiler
unsafe impl Send for Sphere {}
unsafe impl Sync for Sphere {}