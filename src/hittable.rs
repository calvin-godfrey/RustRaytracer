use nalgebra::base::{Unit, Vector3, Vector2, Matrix4};
use nalgebra::geometry::Point3;
use std::cmp::Ordering;
use crate::material::materials::Material;
use crate::geometry::Ray;
use crate::parser;
use crate::util;

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
    fn get_bounding_box(&self, t0: f64, t1: f64) -> Option<BoundingBox>;
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

    #[allow(unused_variables)]
    fn get_bounding_box(&self, t0: f64, t1: f64) -> Option<BoundingBox> {
        let r_vector = Vector3::new(self.r, self.r, self.r);
        Some(BoundingBox::new(self.center - r_vector, self.center + r_vector))
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
    pub mat: Box<dyn Material>,
}

impl Hittable for Mesh {
    fn intersects(&self, ray: &Ray, tmin: f64, tmax: f64) -> Option<HitRecord> {
        if let Some(bbox) = &self.bounding_box {
            if !bbox.intersects(ray, tmin, tmax) { return None; }
        }
        let records: Vec<HitRecord> = (0..self.ind.len()).step_by(3).filter_map(|ind| self.intersects_triangle(ray, ind, tmin, tmax)).collect();
        let rec = records.iter().min_by(|x, y| x.t.partial_cmp(&y.t).unwrap());
        match rec {
            Some(r) => return Some(*r),
            None => return None
        }
    }

    #[allow(unused_variables)]
    fn get_bounding_box(&self, t0: f64, f1: f64) -> Option<BoundingBox> {
        match self.bounding_box {
            Some(bbox) => {
                Some(bbox.clone())
            },
            None => None
        }
    }
}

impl Mesh {
    pub fn new(path: &str, trans: Matrix4<f64>) -> Self {
        parser::parse_obj(path, trans)
    }

    pub fn generate_triangles(mesh: &Self) -> Vec<Triangle> {
        let mut triangles: Vec<Triangle> = Vec::new();
        for index in (0..mesh.ind.len()).step_by(3) {
            let v1 = mesh.p[mesh.ind[index]];
            let v2 = mesh.p[mesh.ind[index + 1]];
            let v3 = mesh.p[mesh.ind[index + 2]];
            let x_min = v1.x.min(v2.x.min(v3.x));
            let x_max = v1.x.max(v2.x.max(v3.x));
            let y_min = v1.y.min(v2.y.min(v3.y));
            let y_max = v1.y.max(v2.y.max(v3.y));
            let z_min = v1.z.min(v2.y.min(v3.z));
            let z_max = v1.z.max(v2.y.max(v3.z));
            let tri_box = BoundingBox::new(Point3::new(x_min, y_min, z_min), Point3::new(x_max, y_max, z_max));
            let tri: Triangle = Triangle::new(index, &mesh, Some(tri_box));
            triangles.push(tri);
        }
        triangles
    }

    fn intersects_triangle(&self, ray: &Ray, ind: usize, tmin: f64, tmax: f64) -> Option<HitRecord> {
        let ind1 = self.ind[ind];
        let ind2 = self.ind[ind + 1];
        let ind3 = self.ind[ind + 2];
        let p0 = self.p[ind1];
        let p1 = self.p[ind2];
        let p2 = self.p[ind3];
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
            let n1 = (1. - u - v) * self.n[ind1];
            let n2 = u * self.n[ind2];
            let n3 = v * self.n[ind3];
            n1 + n2 + n3
        };
        let normal = Unit::new_normalize(normal);
        let point = ray.at(t);

        let uv: Option<Vector2<f64>> = if self.uv.len() == 0 {
            None
        } else {
            let uv1 = ( 1. - u - v) * self.uv[ind1];
            let uv2 = u * self.uv[ind2];
            let uv3 = v * self.uv[ind3];
            Some(uv1 + uv2 + uv3)
        };

        let mut record = HitRecord::new(t, normal, point, true, &self.mat);
        record.set_front(ray);
        record.uv = uv;
        Some(record)
    }
}

pub struct Triangle <'b> {
    id: usize,
    mesh: &'b Mesh,
    bounding_box: Option<BoundingBox>
}

impl<'b> Hittable for Triangle<'b> {
    fn intersects(&self, ray: &Ray, tmin: f64, tmax: f64) -> Option<HitRecord> {
        self.mesh.intersects_triangle(ray, self.id, tmin, tmax)
    }

    #[allow(unused_variables)]
    fn get_bounding_box(&self, t0: f64, f1: f64) -> Option<BoundingBox> {
        match self.bounding_box {
            Some(bbox) => {
                Some(bbox.clone())
            },
            None => None
        }
    }
}

impl<'b> Triangle<'b> {
    pub fn new(id: usize, mesh: &'b Mesh, bounding_box: Option<BoundingBox>) -> Self { Self { id, mesh, bounding_box } }
}

#[derive(Copy, Clone)]
pub struct BoundingBox {
    pub min: Point3<f64>,
    pub max: Point3<f64>,
}

impl BoundingBox {
    pub fn new(min: Point3<f64>, max: Point3<f64>) -> Self { Self { min, max } }
    pub fn intersects(&self, ray: &Ray, tmi: f64, tma: f64) -> bool {
        let mut tmin = tmi;
        let mut tmax = tma;
        for a in 0..3 {
            let inv_d = 1. / ray.dir[a];
            let val1 = (self.min[a] - ray.origin[a]) * inv_d;
            let val2 = (self.max[a] - ray.origin[a]) * inv_d;
            let t0: f64 = val1.min(val2);
            let t1: f64 = val1.max(val2);
            tmin = tmin.max(t0);
            tmax = tmax.min(t1);
            if tmax <= tmin {
                return false;
            }
        };
        true
    }
    pub fn union(bbox1: &Self, bbox2: &Self) -> Self {
        let min: Point3<f64> = Point3::new(
            bbox1.min.x.min(bbox2.min.x),
            bbox1.min.x.min(bbox2.min.y),
            bbox1.min.z.min(bbox2.min.z)
        );
        let max: Point3<f64> = Point3::new(
            bbox1.min.x.max(bbox2.max.x),
            bbox1.min.x.max(bbox2.max.y),
            bbox1.min.z.max(bbox2.max.z)
        );
        Self::new(min, max)
    }
}

pub struct MovingSphere {
    r: f64,
    mat: Box<dyn Material>,
    t0: f64,
    t1: f64,
    c0: Point3<f64>,
    c1: Point3<f64>,
}

impl MovingSphere {
    pub fn new(c0: Point3<f64>, c1: Point3<f64>, t0: f64, t1: f64, r: f64, mat: Box<dyn Material>) -> Self { Self { r, mat, t0, t1, c0, c1 } }
    pub fn center(&self, time: f64) -> Point3<f64> {
        let diff: Vector3<f64> = self.c1 - self.c0;
        self.c0 + (time - self.t0) / (self.t1 - self.t0) * diff
    }
}

impl Hittable for MovingSphere {
    fn intersects(&self, ray: &Ray, tmin: f64, tmax: f64) -> Option<HitRecord> {
        let center = self.center(ray.time);
        let diff: Vector3<f64> = ray.origin - center;
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
            let mut hit_record = HitRecord::new(ans, Unit::new_normalize(hit - center), hit, true, &self.mat);
            hit_record.set_front(ray);
            return Some(hit_record);
        }
        let ans = (-b + root) * inv_a;
        if ans < tmax && ans > tmin {
            let hit = ray.at(ans);
            let mut hit_record = HitRecord::new(ans, Unit::new_normalize(hit - center), hit, true, &self.mat);
            hit_record.set_front(ray);
            return Some(hit_record);
        } else {
            return None;
        }
    }
    fn get_bounding_box(&self, t0: f64, t1: f64) -> Option<BoundingBox> {
        let c0 = self.center(t0);
        let c1 = self.center(t1);
        let r_vector = Vector3::new(self.r, self.r, self.r);
        let bbox1 = BoundingBox::new(c0 - r_vector, c0 + r_vector);
        let bbox2 = BoundingBox::new(c1 - r_vector, c1 + r_vector);
        let bounding_box = BoundingBox::union(&bbox1, &bbox2);
        Some(bounding_box)
    }
}

#[derive(Clone)]
pub struct BvhNode<'c> {
    bounding_box: BoundingBox,
    left: &'c Box<dyn Hittable>,
    right: &'c Box<dyn Hittable>,
}

impl<'c> Hittable for BvhNode<'c> {
    fn intersects(&self, ray: &Ray, tmin: f64, tmax: f64) -> Option<HitRecord> {
        if !self.bounding_box.intersects(ray, tmin, tmax) {
            return None;
        }
        
        let left_option = self.left.intersects(ray, tmin, tmax);
        let right_option = self.right.intersects(ray, tmin, tmax);

        match left_option {
            Some(l) => {
                match right_option {
                    Some(r) => return if l.t < r.t { left_option } else { right_option },
                    None => return left_option
                }
            },
            None => {
                match right_option {
                    Some(_) => return right_option,
                    None => return None
                }
            }
        }

    }

    #[allow(unused_variables)]
    fn get_bounding_box(&self, t0: f64, f1: f64) -> Option<BoundingBox> {
        return Some(self.bounding_box.clone());
    }
    
}

impl<'c> BvhNode<'c> {
    pub fn new(objects: Vec<Box<dyn Hittable>>, start: usize, end: usize, t0: f64, t1: f64) -> Self {
        let r = util::rand();
        let comp = if r < 1. / 3. {
            util::box_x_compare
        } else if r < 2. / 3. {
            util::box_y_compare
        } else {
            util::box_z_compare
        };
        let num_obj = end - start;
        let mut left: &Box<dyn Hittable>;
        let mut right: &Box<dyn Hittable>;
        if num_obj == 1 {
            left = &objects[start];
            right = &objects[start];
        } else if num_obj == 2 {
            if comp(&&objects[start], &&objects[start + 1]) != Ordering::Greater {
                left = &objects[start];
                right = &objects[start + 1];
            } else {
                left = &objects[start + 1];
                right = &objects[start];
            }
        } else {
            let mut slice: Vec<&Box<dyn Hittable>> = Vec::new();
            for i in start..end {
                slice.push(&objects[i]);
            }
            slice.sort_by(comp);
            let mid = start + num_obj / 2;
            let l: dyn Hittable = BvhNode::new(objects, start, mid, t0, t1);
            let r = BvhNode::new(objects, mid, end, t0, t1);
            left = &Box::new(l);
            right = &Box::new(BvhNode::new(objects, mid, end, t0, t1));
        }
        Self { left: Box::new(), right: None, bounding_box: BoundingBox::new(Point3::origin(), Point3::origin()) }
    }
}

// to please compiler. It's needed for multithreading even though these traits are never used
unsafe impl Send for Sphere {}
unsafe impl Sync for Sphere {}
unsafe impl Send for Mesh {}
unsafe impl Sync for Mesh {}
unsafe impl Send for MovingSphere {}
unsafe impl Sync for MovingSphere {}
unsafe impl<'b> Send for Triangle<'b> {}
unsafe impl<'b> Sync for Triangle<'b> {}
unsafe impl<'c> Send for BvhNode<'c> {}
unsafe impl<'c> Sync for BvhNode<'c> {}