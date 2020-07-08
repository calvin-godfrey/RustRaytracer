use nalgebra::base::{Unit, Vector3, Vector2};
use nalgebra::geometry::Point3;
use crate::material::materials::Material;
use crate::geometry::Ray;
use crate::parser;
use crate::consts::*;

#[derive(Copy, Clone)]
pub struct HitRecord<'a> {
    pub t: f64, // time of hit along ray
    pub n: Unit<Vector3<f64>>, // normal of surface at point
    pub p: Point3<f64>, // point of intersection
    pub front: bool, // if the normal points outwards or not
    pub mat: &'a Box<dyn Material>, // how the surface acts
    pub uv: Option<Vector2<f64>>, // uv texture coordinates
}

impl <'a> HitRecord<'a> {
    pub fn new(t: f64, n: Unit<Vector3<f64>>, p: Point3<f64>, front: bool, mat: &'a Box<dyn Material>) -> Self { Self { t, n, p, front, mat, uv: None } }
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

pub struct Mesh {
    // ith triangle has vertices at p[ind[3 * i]], ...
    // and normals at n[ind[3 * i]], ...
    pub ind: Vec<usize>,
    pub p: Vec<Point3<f64>>,
    pub n: Vec<Vector3<f64>>,
    pub uv: Vec<Vector2<f64>>,
    pub bounding_box: Option<BoundingBox>,
    pub triangle_boxes: Vec<BoundingBox>,
    pub mat: Box<dyn Material>,
}

impl Hittable for Mesh {
    fn intersects(&self, ray: &Ray, tmin: f64, tmax: f64) -> Option<HitRecord> {
        if let Some(bbox) = &self.bounding_box {
            if !bbox.intersects(ray) { return None; }
        }
        let records: Vec<HitRecord> = (0..self.ind.len()).step_by(3).filter_map(|ind| self.intersects_triangle(ray, ind, tmin, tmax)).collect();
        let rec = records.iter().min_by(|x, y| x.t.partial_cmp(&y.t).unwrap());
        match rec {
            Some(r) => return Some(*r),
            None => return None
        }
    }
}

impl Mesh {
    pub fn new(path: &str, use_bb: bool) -> Self {
        parser::parse_obj(path, use_bb)
    }

    fn intersects_triangle(&self, ray: &Ray, ind: usize, tmin: f64, tmax: f64) -> Option<HitRecord> {
        if self.triangle_boxes.len() > 0 && !self.triangle_boxes[ind / 3].intersects(ray) { return None; }
        let p0 = self.p[self.ind[ind]];
        let p1 = self.p[self.ind[ind + 1]];
        let p2 = self.p[self.ind[ind + 2]];
        let dir: Vector3<f64> = ray.dir;
        let temp1 = p1 - p0;
        let temp2: Vector3<f64> = p2 - p0;
        let pvec: Vector3<f64> = dir.cross(&temp2);
        let det: f64 = temp1.dot(&pvec);
        let inv_det = 1. / det;
        let tvec = ray.origin - p0;
        let u = pvec.dot(&tvec) * inv_det;

        if u < 0. || u > 1. { return None; }
        let qvec: Vector3<f64> = tvec.cross(&temp1);
        let v = qvec.dot(&dir) * inv_det;

        if v < 0. || u + v > 1. { return None; }
        let t = qvec.dot(&temp2) * inv_det;

        if t < tmin || t > tmax { return None; }
        let normal: Vector3<f64> = if self.n.len() == 0 {
            temp1.cross(&temp2)
        } else { 
            let n1 = self.n[self.ind[ind]].scale(1. - u - v);
            let n2 = self.n[self.ind[ind + 1]].scale(u);
            let n3 = self.n[self.ind[ind + 2]].scale(v);
            n1 + n2 + n3
        };
        let normal = Unit::new_normalize(normal);
        let point = ray.at(t);

        let uv: Option<Vector2<f64>> = if self.uv.len() == 0 {
            None
        } else {
            let uv1 = self.uv[self.ind[ind]].scale(1. - u - v);
            let uv2 = self.uv[self.ind[ind + 1]]; //.scale(v);
            let uv3 = self.uv[self.ind[ind + 2]]; //.scale(u);
            let option1 = uv1 + uv2.scale(v) + uv3.scale(u);
            let option2 = uv1 + uv2.scale(u) + uv3.scale(v);
            // println!("({:.4}, {:.4}), ({:.4}, {:.4})", uv2.x, uv2.y, uv3.x, uv3.y);
            // println!("{:.4}, {:.4} vs. {:.4}, {:.4} ({:.4}, {:.4})", option1.x, option1.y, option2.x, option2.y, u, v);
            Some(option2)
        };

        let mut record = HitRecord::new(t, normal, point, true, &self.mat);
        record.set_front(ray);
        record.uv = uv;
        Some(record)
    }
}

pub struct BoundingBox {
    min: Point3<f64>,
    max: Point3<f64>,
}

impl BoundingBox {
    pub fn new(min: Point3<f64>, max: Point3<f64>) -> Self { Self { min, max } }
    pub fn intersects(&self, ray: &Ray) -> bool {
        let dir = ray.dir;
        let inv = Vector3::new(1. / dir.x, 1. / dir.y, 1. / dir.z);
        let t1: f64 = (self.min.x - ray.origin.x) * inv.x;
        let t2: f64 = (self.max.x - ray.origin.x) * inv.x;
        let tmin: f64 = t1.min(t2);
        let tmax: f64 = t1.max(t2);

        let t1: f64 = (self.min.y - ray.origin.y) * inv.y;
        let t2: f64 = (self.max.y - ray.origin.y) * inv.y;

        let tmin = tmin.max(tmax.min(t1.min(t2)));
        let tmax = tmax.min(tmin.max(t1.max(t2)));

        let t1: f64 = (self.min.z - ray.origin.z) * inv.z;
        let t2: f64 = (self.max.z - ray.origin.z) * inv.z;

        let tmin = tmin.max(tmax.min(t1.min(t2)));
        let tmax = tmax.min(tmin.max(t1.max(t2)));

        tmax > tmin.max(0.)
    }
}

// to please compiler. It's needed for multithreading even though these traits are never used
unsafe impl Send for Sphere {}
unsafe impl Sync for Sphere {}
unsafe impl Send for Mesh {}
unsafe impl Sync for Mesh {}