use nalgebra::base::{Unit, Vector3, Vector2};
use nalgebra::geometry::{Projective3, Point3};
use std::cmp::Ordering;
use std::sync::Arc;
use crate::geometry::Ray;
use crate::material::materials::{Texture, Material};
use crate::parser;
use crate::util;
use crate::primitive::Primitive;
#[derive(Clone)]
pub struct HitRecord {
    pub t: f64, // time of hit along ray
    pub n: Unit<Vector3<f64>>, // normal of surface at point
    pub p: Point3<f64>, // point of intersection
    pub front: bool, // if the normal points outwards or not
    pub uv: Vector2<f64>, // uv texture coordinates
    pub mat_index: usize,
}

impl HitRecord {
    pub fn new(t: f64, n: Unit<Vector3<f64>>, p: Point3<f64>, front: bool, mat_index: usize) -> Self { Self { t, n, p, front, mat_index, uv: Vector2::new(0., 0.) } }
    pub fn set_front(&mut self, ray: &Ray) {
        self.front = ray.dir.dot(self.n.as_ref()) < 0.0;
        self.n = if self.front { self.n } else { -self.n }
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
    pub mat_index: usize,
}

impl Mesh {
    pub fn new(materials: &mut Vec<Material>, textures: &mut Vec<Texture>, path: &str, trans: Projective3<f64>) -> Self {
        parser::parse_obj(materials, textures, path, trans)
    }

    pub fn generate_triangles(mesh: &Arc<Self>) -> Vec<Primitive> {
        let mut triangles: Vec<Primitive> = Vec::new();
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
            let tri: Primitive = Primitive::Triangle {mesh: Arc::clone(mesh), ind: index, bounding_box: Some(tri_box)};
            triangles.push(tri);
        }
        triangles
    }

    pub fn intersects_triangle(&self, ray: &Ray, ind: usize, tmin: f64, tmax: f64) -> Option<HitRecord> {
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

        let mut record = HitRecord::new(t, normal, point, true, self.mat_index);
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
        index: usize,
        bounding_box: BoundingBox,
    },
    Empty
}

impl BvhNode {
    pub fn intersects(&self, objs: &Vec<Primitive>, ray: &Ray, tmin: f64, tmax: f64) -> Option<HitRecord> {
        match self {
            BvhNode::Internal { left, right, bounding_box } => {
                if !bounding_box.intersects(ray, tmin, tmax) {
                    return None;
                }
                let left_option = left.intersects(objs, ray, tmin, tmax);
                let right_option = right.intersects(objs, ray, tmin, tmax);

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
            BvhNode::Leaf { index, bounding_box } => {
                if !bounding_box.intersects(ray, tmin, tmax) {
                    return None;
                }
                return Primitive::intersects(objs, *index, ray, tmin, tmax);
            }
            BvhNode::Empty => { return None; }
        }
    }

    #[allow(unused_variables)]
    pub fn new(objects: &mut Vec<Primitive>, start: usize, end: usize, t0: f64, t1: f64) -> BvhNode {
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
        if num_obj == 1 {
            let bbox = Primitive::get_bounding_box(&objects[start], t0, t1);
            let bbox_copy = BoundingBox::make_copy(bbox);
            return BvhNode::Leaf {index: start, bounding_box: bbox_copy};
        } else if num_obj == 2 {
            if comp(&objects[start], &objects[start + 1]) != Ordering::Greater {
                return BvhNode::handle_two(objects, start, start + 1, t0, t1);
            } else {
                return BvhNode::handle_two(objects, start + 1, start, t0, t1);
            }
        } else {
            let slice = &mut objects[start..end];
            slice.sort_by(comp);
            let mid = start + num_obj / 2;
            final_left = BvhNode::new(objects, start, mid, t0, t1);
            final_right = BvhNode::new(objects, mid, end, t0, t1);
        }

        let left_box: Option<BoundingBox> = match &final_left {
            BvhNode::Internal { left, right, bounding_box } => {Some(BoundingBox::make_new(bounding_box))}
            BvhNode::Leaf { index, bounding_box } => {Some(BoundingBox::make_new(bounding_box))}
            BvhNode::Empty => { None }
        };

        let right_box: Option<BoundingBox> = match &final_right {
            BvhNode::Internal { left, right, bounding_box } => {Some(BoundingBox::make_new(bounding_box))}
            BvhNode::Leaf { index, bounding_box } => {Some(BoundingBox::make_new(bounding_box))}
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

    fn handle_two(objs: &Vec<Primitive>, curr: usize, next: usize, t0: f64, t1: f64) -> BvhNode {
        let inner_bb = BoundingBox::make_copy(Primitive::get_bounding_box(&objs[next], t0, t1));
        let curr_bb = BoundingBox::make_copy(Primitive::get_bounding_box(&objs[curr], t0, t1));
        let bb = BoundingBox::union(&inner_bb, &curr_bb);
        let inner_right = BvhNode::Leaf {index: curr, bounding_box: curr_bb };
        let inner_left = BvhNode::Leaf { index: next, bounding_box: inner_bb };

        BvhNode::Internal {left: Box::new(inner_left), right: Box::new(inner_right), bounding_box: bb}
    }
}

pub struct Cube {
    mat_index: usize,
    min: Vector3<f64>,
    max: Vector3<f64>,
    transform: Option<Arc<Projective3<f64>>>,
}

impl Cube {
    #[allow(dead_code)]
    pub fn new(min: Vector3<f64>, max: Vector3<f64>, mat_index: usize) -> Self {
        Self {min, max, mat_index, transform: None }
    }

    pub fn new_transform(min: Vector3<f64>, max: Vector3<f64>, mat_index: usize, transform: Arc<Projective3<f64>>) -> Self {
        Self { min, max, mat_index, transform: Some(Arc::clone(&transform))}
    }

    pub fn get_sides(&self) -> Vec<Primitive> {
        let min = self.min;
        let max = self.max;
        let z0 = Primitive::new_flip_face(Box::new(Primitive::new_xy_rect_transform(min.x, min.y, max.x, max.y, min.z, self.mat_index, self.transform.clone())));
        let z1 = Primitive::new_xy_rect_transform(min.x, min.y, max.x, max.y, max.z, self.mat_index, self.transform.clone());
        let x0 = Primitive::new_flip_face(Box::new(Primitive::new_yz_rect_transform(min.y, min.z, max.y, max.z, min.x, self.mat_index, self.transform.clone())));
        let x1 = Primitive::new_yz_rect_transform(min.y, min.z, max.y, max.z, max.x, self.mat_index, self.transform.clone());
        let y0 = Primitive::new_flip_face(Box::new(Primitive::new_xz_rect_transform(min.x, min.z, max.x, max.z, min.y, self.mat_index, self.transform.clone())));
        let y1 = Primitive::new_xz_rect_transform(min.x, min.z, max.x, max.z, max.y, self.mat_index, self.transform.clone());
        vec![z0, z1, y0, y1, x0, x1]
    }
}

// to please compiler. It's needed for multithreading even though these traits are never used
// unsafe impl Send for Primitive {}
// unsafe impl Sync for Primitive {}
// unsafe impl Send for BvhNode {}
// unsafe impl Sync for BvhNode {}