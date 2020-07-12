use nalgebra::base::{Unit, Vector3, Vector2, Matrix4};
use nalgebra::geometry::Point3;
use std::cmp::Ordering;
use std::sync::Arc;
use crate::material::materials::Material;
use crate::geometry::Ray;
use crate::parser;
use crate::util;

#[derive(Clone)]
pub struct HitRecord {
    pub t: f64, // time of hit along ray
    pub n: Unit<Vector3<f64>>, // normal of surface at point
    pub p: Point3<f64>, // point of intersection
    pub front: bool, // if the normal points outwards or not
    pub mat: Arc<Material>, // how the surface acts
    pub uv: Vector2<f64>, // uv texture coordinates
}

impl HitRecord {
    pub fn new(t: f64, n: Unit<Vector3<f64>>, p: Point3<f64>, front: bool, mat: Arc<Material>) -> Self { Self { t, n, p, front, mat, uv: Vector2::new(0., 0.) } }
    pub fn set_front(&mut self, ray: &Ray) {
        self.front = ray.dir.dot(self.n.as_ref()) < 0.0;
        self.n = if self.front { self.n } else { -self.n }
    }
}

pub enum Primitive {
    Sphere {
        center: Point3<f64>,
        r: f64,
        mat: Arc<Material>,
        bounding_box: Option<BoundingBox>,
    },
    Triangle {
        mesh: Arc<Mesh>,
        ind: usize,
        bounding_box: Option<BoundingBox>,
    },
    MovingSphere {
        r: f64,
        mat: Arc<Material>,
        t0: f64,
        t1: f64,
        c0: Point3<f64>,
        c1: Point3<f64>,
        bounding_box: Option<BoundingBox>,
    }
}

impl Primitive {
    pub fn new_sphere(center: Point3<f64>, r: f64, mat: Arc<Material>) -> Self {
        let r_vec = Vector3::new(r, r, r);
        let min = center - r_vec;
        let max = center + r_vec;
        let bounding_box = Some(BoundingBox::new(min, max));
        Primitive::Sphere {center, r, mat, bounding_box }
    }

    #[allow(unused_variables)]
    pub fn intersects(obj: &Primitive, ray: &Ray, tmin: f64, tmax: f64) -> Option<HitRecord> {
        match obj {
            Primitive::Sphere { center, r, mat, bounding_box } => {sphere_intersect(center, r, mat, ray, tmin, tmax)}
            Primitive::Triangle { mesh, ind, bounding_box } => {mesh.intersects_triangle(ray, *ind, tmin, tmax)}
            Primitive::MovingSphere { r, mat, t0, t1, c0, c1, bounding_box } => {moving_sphere_intersect(*r, mat, *t0, *t1, c0, c1, ray, tmin, tmax)}
        }
    }

    #[allow(unused_variables)]
    pub fn get_bounding_box(obj: &Primitive, time0: f64, time1: f64) -> &Option<BoundingBox> {
        match obj {
            Primitive::Sphere { center, r, mat, bounding_box } => {
                bounding_box
            }
            Primitive::Triangle { mesh, ind, bounding_box } => {bounding_box}
            Primitive::MovingSphere { r, mat, t0, t1, c0, c1, bounding_box } => {
                bounding_box
            }
        }
    }
}

pub fn make_sphere(center: Point3<f64>, r: f64, mat: Arc<Material>) -> Primitive {
    Primitive::new_sphere(center, r, mat)
}

pub fn make_moving_sphere(c0: Point3<f64>, c1: Point3<f64>, t0: f64, t1: f64, r: f64, mat: Arc<Material>) -> Primitive {
    let c0 = moving_sphere_center(&c0, &c1, t0, t1, t0);
    let c1 = moving_sphere_center(&c0, &c1, t0, t1, t1);
    let min0 = Point3::new(c0.x - r, c0.y - r, c0.z - r);
    let max0 = Point3::new(c0.x + r, c0.y + r, c0.z + r);
    let min1 = Point3::new(c1.x - r, c1.y - r, c1.z - r);
    let max1 = Point3::new(c1.x + r, c1.y + r, c1.z + r);
    let bbox1 = BoundingBox::new(min0, max0);
    let bbox2 = BoundingBox::new(min1, max1);
    let bounding_box = BoundingBox::union(&bbox1, &bbox2);
    Primitive::MovingSphere { c0, c1, t0, t1, r, mat, bounding_box: Some(bounding_box) }
}

fn sphere_intersect(center: &Point3<f64>, r: &f64, mat: &Arc<Material>, ray: &Ray, tmin: f64, tmax: f64) -> Option<HitRecord> {
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
        let ans = (-b - root) * inv_a; // try first solution to equationd
        let mut hit_record: HitRecord;
        if ans < tmax && ans > tmin {
            let hit = ray.at(ans);
            hit_record = HitRecord::new(ans, Unit::new_normalize(hit - center), hit, true, Arc::clone(&mat));
            hit_record.set_front(ray);
        } else {
            let ans = (-b + root) * inv_a;
            if ans < tmax && ans > tmin {
                let hit = ray.at(ans);
                let mut hit_record = HitRecord::new(ans, Unit::new_normalize(hit - center), hit, true, Arc::clone(&mat));
                hit_record.set_front(ray);
                return Some(hit_record);
            } else {
                return None;
            }
        }
        util::get_sphere_uv((hit_record.p - center).scale(*r), &mut hit_record);
        Some(hit_record)
}

fn moving_sphere_intersect(r: f64, mat: &Arc<Material>, t0: f64, t1: f64, c0: &Point3<f64>, c1: &Point3<f64>, ray: &Ray, tmin: f64, tmax: f64) -> Option<HitRecord> {
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
        let mut hit_record = HitRecord::new(ans, Unit::new_normalize(hit - center), hit, true, Arc::clone(mat));
        hit_record.set_front(ray);
        return Some(hit_record);
    }
    let ans = (-b + root) * inv_a;
    if ans < tmax && ans > tmin {
        let hit = ray.at(ans);
        let mut hit_record = HitRecord::new(ans, Unit::new_normalize(hit - center), hit, true, Arc::clone(mat));
        hit_record.set_front(ray);
        return Some(hit_record);
    } else {
        return None;
    }
}

fn moving_sphere_center(c0: &Point3<f64>, c1: &Point3<f64>, t0: f64, t1: f64, time: f64) -> Point3<f64> {
    let diff: Vector3<f64> = c1 - c0;
    c0 + (time - t0) / (t1 - t0) * diff
}

pub struct Mesh {
    // ith triangle has vertices at p[ind[3 * i]], ...
    // and normals at n[ind[3 * i]], ...
    pub ind: Vec<usize>,
    pub p: Vec<Point3<f64>>,
    pub n: Vec<Vector3<f64>>,
    pub uv: Vec<Vector2<f64>>,
    pub bounding_box: Option<BoundingBox>,
    pub mat: Arc<Material>,
}

impl Mesh {
    pub fn new(path: &str, trans: Matrix4<f64>) -> Self {
        parser::parse_obj(path, trans)
    }

    pub fn generate_triangles(mesh: &Arc<Self>) -> Vec<Box<Primitive>> {
        let mut triangles: Vec<Box<Primitive>> = Vec::new();
        for index in (0..mesh.ind.len()).step_by(3) {
            let v1 = mesh.p[mesh.ind[index]];
            let v2 = mesh.p[mesh.ind[index + 1]];
            let v3 = mesh.p[mesh.ind[index + 2]];
            let x_min = v1.x.min(v2.x.min(v3.x));
            let x_max = v1.x.max(v2.x.max(v3.x));
            let y_min = v1.y.min(v2.y.min(v3.y));
            let y_max = v1.y.max(v2.y.max(v3.y));
            let z_min = v1.z.min(v2.z.min(v3.z));
            let z_max = v1.z.max(v2.z.max(v3.z));
            let tri_box = BoundingBox::new(Point3::new(x_min, y_min, z_min), Point3::new(x_max, y_max, z_max));
            let tri: Box<Primitive> = Box::new(Primitive::Triangle {mesh: Arc::clone(mesh), ind: index, bounding_box: Some(tri_box)});
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

        let uv: Vector2<f64> = if self.uv.len() == 0 {
            Vector2::new(0., 0.) // dummy value
        } else {
            let uv1 = ( 1. - u - v) * self.uv[ind1];
            let uv2 = u * self.uv[ind2];
            let uv3 = v * self.uv[ind3];
            uv1 + uv2 + uv3
        };

        let mut record = HitRecord::new(t, normal, point, true, Arc::clone(&self.mat));
        record.set_front(ray);
        record.uv = uv;
        Some(record)
    }
}

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
            bbox1.min.y.min(bbox2.min.y),
            bbox1.min.z.min(bbox2.min.z)
        );
        let max: Point3<f64> = Point3::new(
            bbox1.max.x.max(bbox2.max.x),
            bbox1.max.y.max(bbox2.max.y),
            bbox1.max.z.max(bbox2.max.z)
        );
        Self::new(min, max)
    }

    pub fn make_copy(bounding_box: &Option<BoundingBox>) -> BoundingBox {
        match bounding_box {
            Some(x) => {
                BoundingBox::new(x.min.clone(), x.max.clone())
            },
            None => panic!("Can't copy a None bounding box")
        }
    }

    pub fn make_new(bounding_box: &BoundingBox) -> BoundingBox {
        BoundingBox::new(bounding_box.min.clone(), bounding_box.max.clone())
    }
}

pub enum BvhNode {
    Internal {
        left: Box<BvhNode>,
        right: Box<BvhNode>,
        bounding_box: BoundingBox,
    },
    Leaf {
        obj: Box<Primitive>,
        bounding_box: BoundingBox,
    },
    Empty
}

impl BvhNode {
    pub fn intersects(&self, ray: &Ray, tmin: f64, tmax: f64) -> Option<HitRecord> {
        match self {
            BvhNode::Internal { left, right, bounding_box } => {
                if !bounding_box.intersects(ray, tmin, tmax) {
                    return None;
                }
                let left_option = left.intersects(ray, tmin, tmax);
                let right_option = right.intersects(ray, tmin, tmax);

                match &left_option {
                    Some(l) => {
                        match &right_option {
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
            BvhNode::Leaf { obj, bounding_box } => {
                if !bounding_box.intersects(ray, tmin, tmax) {
                    return None;
                }
                return Primitive::intersects(obj, ray, tmin, tmax);
            }
            BvhNode::Empty => { return None; }
        }
    }

    #[allow(unused_variables)]
    pub fn new(objects: &mut Vec<Box<Primitive>>, start: usize, end: usize, t0: f64, t1: f64) -> BvhNode {
        let r = util::rand();
        let comp = if r < 1. / 3. {
            util::box_x_compare
        } else if r < 2. / 3. {
            util::box_y_compare
        } else {
            util::box_z_compare
        };
        let num_obj = end - start;
        let final_left: BvhNode;
        let final_right: BvhNode;
        let curr: Box<Primitive>;
        if num_obj == 1 {
            curr = objects.remove(0);
            let bbox = Primitive::get_bounding_box(curr.as_ref(), t0, t1);
            let bbox_copy = BoundingBox::make_copy(bbox);
            return BvhNode::Leaf {obj: curr, bounding_box: bbox_copy};
        } else if num_obj == 2 {
            if comp(&objects[start], &objects[start + 1]) != Ordering::Greater {
                curr = objects.remove(0);
                let next = objects.remove(0);
                return BvhNode::handle_two(curr, next, t0, t1);
            } else {
                let next = objects.remove(0);
                curr = objects.remove(0);
                return BvhNode::handle_two(curr, next, t0, t1);
            }
        } else {
            let slice = &mut objects[start..end];
            slice.sort_by(comp);
            let mid = start + num_obj / 2;
            final_left = BvhNode::new(objects, start, mid, t0, t1);
            final_right = BvhNode::new(objects, start, end - mid, t0, t1);
        }

        let left_box: Option<BoundingBox> = match &final_left {
            BvhNode::Internal { left, right, bounding_box } => {Some(BoundingBox::make_new(bounding_box))}
            BvhNode::Leaf { obj, bounding_box } => {Some(BoundingBox::make_new(bounding_box))}
            BvhNode::Empty => { None }
        };

        let right_box: Option<BoundingBox> = match &final_right {
            BvhNode::Internal { left, right, bounding_box } => {Some(BoundingBox::make_new(bounding_box))}
            BvhNode::Leaf { obj, bounding_box } => {Some(BoundingBox::make_new(bounding_box))}
            BvhNode::Empty => { None }
        };

        let curr_box: Option<BoundingBox> = match left_box {
            Some(x) => {
                match right_box {
                    Some(y) => {
                        Some(BoundingBox::union(&x, &y))
                    },
                    None => Some(x)
                }
            }
            None => {
                match right_box {
                    Some(y) => {
                        Some(y)
                    },
                    None => None
                }
            }
        };

        if curr_box.is_none() { // shouldn't happen
            return BvhNode::Empty;
        }
        BvhNode::Internal { left: Box::new(final_left), right: Box::new(final_right), bounding_box: curr_box.unwrap() }
    }

    fn handle_two(curr: Box<Primitive>, next: Box<Primitive>, t0: f64, t1: f64) -> BvhNode {
        let inner_bb = BoundingBox::make_copy(Primitive::get_bounding_box(next.as_ref(), t0, t1));
        let curr_bb = BoundingBox::make_copy(Primitive::get_bounding_box(curr.as_ref(), t0, t1));
        let bb = BoundingBox::union(&inner_bb, &curr_bb);
        let inner_right = BvhNode::Leaf {obj: curr, bounding_box: curr_bb };
        let inner_left = BvhNode::Leaf { obj: next, bounding_box: inner_bb };

        BvhNode::Internal {left: Box::new(inner_left), right: Box::new(inner_right), bounding_box: bb}
    }
}

// to please compiler. It's needed for multithreading even though these traits are never used
unsafe impl Send for Primitive {}
unsafe impl Sync for Primitive {}
unsafe impl Send for Mesh {}
unsafe impl Sync for Mesh {}
unsafe impl Send for BvhNode {}
unsafe impl Sync for BvhNode {}