use nalgebra::base::{Unit, Vector3, Vector2};
use nalgebra::geometry::{Projective3, Point3};

use std::cmp::Ordering;
use std::sync::Arc;
use crate::geometry::{Ray, get_objects};
use crate::parser;
use crate::util;
use crate::primitive::Primitive;
use crate::consts::*;
use crate::bsdf::Bsdf;
/**
Represents a potential ray *from* p0 *to* p1
*/
pub struct Visibility<'a> {
    pub p0: &'a HitRecord,
    pub p1: HitRecord,
}

impl <'a> Visibility<'a> {
    pub fn make_visibility(p0: &'a HitRecord, p1: HitRecord) -> Self {
        Visibility { p0, p1 }
    }

    pub fn unoccluded(&self) -> bool {
        let dir = self.p1.p - self.p0.p;
        let ray = Ray::new_time(self.p0.p + dir * SMALL, dir, self.p1.t);
        // TODO: Use quicker intersection tests that don't make full record
        let new_record = crate::geometry::get_objects().node.intersects(&ray, 0., INFINITY, 0).unwrap();
        // check that the index matches
        new_record.prim_index == self.p1.prim_index
    }
}


#[derive(Clone, Copy)]
pub struct Shading {
    pub n: Unit<Vector3<f64>>,
    pub dpdu: Unit<Vector3<f64>>,
    pub dpdv: Unit<Vector3<f64>>,
    dndu: Vector3<f64>,
    dndv: Vector3<f64>
}
pub struct HitRecord {
    pub t: f64, // time of hit along ray
    pub n: Unit<Vector3<f64>>, // normal of surface at point
    pub p: Point3<f64>, // point of intersection
    pub front: bool, // if the normal points outwards or not
    pub uv: Vector2<f64>, // uv texture coordinates
    pub mat_index: usize,
    pub prim_index: usize,
    // differential data
    pub dpdu: Vector3<f64>,
    pub dpdv: Vector3<f64>,
    pub dndu: Vector3<f64>,
    pub dndv: Vector3<f64>,
    pub shading: Shading,
    pub wo: Vector3<f64>, // -ray.dir
    pub dpdx: Vector3<f64>, // pixel space. TODO: use these
    pub dpdy: Vector3<f64>,
    pub dudx: Vector3<f64>,
    pub dvdx: Vector3<f64>,
    pub dudy: Vector3<f64>,
    pub dvdy: Vector3<f64>,
    pub bsdf: Bsdf
}

impl HitRecord {
    pub fn new(p: Point3<f64>, uv: Vector2<f64>, wo: Vector3<f64>,
               dpdu: Vector3<f64>, dpdv: Vector3<f64>,
               dndu: Vector3<f64>, dndv: Vector3<f64>, t: f64, prim_index: usize, mat_index: usize) -> Self {
        let n = Unit::new_normalize(dpdu.cross(&dpdv));
        let shading = Shading { n, dpdu: Unit::new_normalize(dpdu), dpdv: Unit::new_normalize(dpdv), dndu, dndv };
        HitRecord { p, n, t, front: false, uv, mat_index, dpdu, dpdv, dndu, dndv, wo, shading, prim_index,
                    dpdx: util::black(), dpdy: util::black(), dudx: util::black(), dvdx: util::black(), dudy: util::black(), dvdy: util::black(), bsdf: Bsdf::empty() }
    }

    pub fn make_basic(p: Point3<f64>, t: f64) -> Self {
        let b = util::black();
        let ub = Unit::new_normalize(b);
        let shading = Shading { n: ub, dpdu: ub, dpdv: ub, dndu: b, dndv: b };
        HitRecord {p, n: ub, t, front: false, uv: Vector2::new(0., 0.), mat_index: 0, dpdu: b, dpdv: b, prim_index: 0,
                    dndu: b, dndv: b, wo: b, shading, dpdx: b, dpdy: b, dudx: b, dvdx: b, dudy: b, dvdy: b, bsdf: Bsdf::empty() }
    }

    pub fn make_normal(p: Point3<f64>, n: Unit<Vector3<f64>>) -> Self {
        let b = util::black();
        let shading = Shading { n: n, dpdu: n, dpdv: n, dndu: b, dndv: b };
        HitRecord {p, n, t: 0f64, front: false, uv: Vector2::new(0., 0.), mat_index: 0, dpdu: b, dpdv: b, prim_index: 0,
            dndu: b, dndv: b, wo: b, shading, dpdx: b, dpdy: b, dudx: b, dvdx: b, dudy: b, dvdy: b, bsdf: Bsdf::empty() }
    }

    pub fn set_front(&mut self, ray: &Ray) {
        self.front = ray.dir.dot(self.n.as_ref()) < 0.0;
        self.n = if self.front { self.n } else { -self.n }
    }

    pub fn set_shading_geometry(&mut self, dpdus: Unit<Vector3<f64>>, dpdvs: Unit<Vector3<f64>>, dndus: Vector3<f64>, dndvs: Vector3<f64>, is_auth: bool) {
        let n = Unit::new_normalize(dpdus.cross(&dpdvs));
        self.shading.n = n;
        if is_auth {
            self.n = util::face_forward(self.n, &self.shading.n);
        } else {
            self.shading.n = util::face_forward(self.shading.n, &self.n);
        }
        self.shading.dpdu = dpdus;
        self.shading.dpdv = dpdvs;
        self.shading.dndu = dndus;
        self.shading.dndv = dndvs;
    }

    /**
    Returns the emitted radiance at a surface point (if any)
    */
    pub fn le(&self, w: &Vector3<f64>) -> Vector3<f64> {
        let objects = get_objects();
        let primitives = &objects.objs;
        let lights = &objects.lights;
        let prim = &primitives[self.prim_index];
        if prim.get_light_index() == std::usize::MAX {
            util::black()
        } else {
            lights[prim.get_light_index()].l(self, w)
        }
    }

    /**
    Generate a ray in the given direction
    */
    pub fn spawn_ray(&self, d: &Vector3<f64>) -> Ray {
        Ray::new_time(self.p, *d, self.t)
    }

    /**
    Generate ray towards the given point
    */
    pub fn spawn_ray_to(&self, p: &Point3<f64>) -> Ray {
        let d = p - self.p;
        Ray::new_time(self.p, d, self.t)
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
}

impl Mesh {
    pub fn new(path: &str, trans: Projective3<f64>, id: usize) -> Self {
        parser::parse_obj(path, trans)
    }

    pub fn generate_triangles(meshes: &Vec<Mesh>, mesh_index: usize, mat_index: usize) -> Vec<Primitive> {
        let mut triangles: Vec<Primitive> = Vec::new();
        let mesh = &meshes[mesh_index];
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
            let tri: Primitive = Primitive::Triangle {mesh_index, ind: index, bounding_box: Some(tri_box), light_index: std::usize::MAX, mat_index};
            triangles.push(tri);
        }
        triangles
    }

    #[warn(non_snake_case)]
    #[allow(unused_variables, unused_assignments)]
    pub fn intersects_triangle(&self, ray: &Ray, ind: usize, mat_index: usize, tmin: f64, tmax: f64) -> Option<HitRecord> {
        let ind1 = self.ind[ind];
        let ind2 = self.ind[ind + 1];
        let ind3 = self.ind[ind + 2];
        let p0 = self.p[ind1];
        let p1 = self.p[ind2];
        let p2 = self.p[ind3];
        let dir: Vector3<f64> = ray.dir;
        // transform vertices so that ray starts at (0, 0, 0) and goes along z+,
        // assuming that vertices start in world space
        let origin_vec = ray.origin.coords; 
        let p0t = p0 - origin_vec;
        let p1t = p1 - origin_vec;
        let p2t = p2 - origin_vec;
        let (kz, _) = dir.abs().argmax();
        let kx = (kz + 1) % 3;
        let ky = (kx + 1) % 3;
        let d = util::permute_vec(&dir, kx, ky, kz);
        let mut p0t = util::permute_pt(&p0t, kx, ky, kz);
        let mut p1t = util::permute_pt(&p1t, kx, ky, kz);
        let mut p2t = util::permute_pt(&p2t, kx, ky, kz);
        // calculate sheer transform; TODO: Precompute some of this?
        let s_x = -d.x / d.z;
        let s_y = -d.y / d.z;
        let s_z = 1. / d.z;
        p0t.x += s_x * p0t.z;
        p0t.y += s_y * p0t.z;
        p1t.x += s_x * p1t.z;
        p1t.y += s_y * p1t.z;
        p2t.x += s_x * p2t.z;
        p2t.y += s_y * p2t.z;
        // wait to shear z components until after intersection is confirmed
        // compute edge coefficients
        let e0 = p1t.x * p2t.y - p1t.y * p2t.x;
        let e1 = p2t.x * p0t.y - p2t.y * p0t.x;
        let e2 = p0t.x * p1t.y - p0t.y * p1t.x;
        // if they aren't all on the same side of the three edges
        if (e0 < 0. || e1 < 0. || e2 < 0.) && (e0 > 0. || e1 > 0. || e2 > 0.) {
            return None; // no intersect
        }
        let det = e0 + e1 + e2;
        if det.abs() < SMALL / 10000. { // hit directly on edge; this will hit another triangle
            return None;
        }
        // apply shear tarnsform to z, calculate barycentric coordinates
        p0t.z *= s_z;
        p1t.z *= s_z;
        p2t.z *= s_z;
        let t_scaled = e0 * p0t.z + e1 * p1t.z + e2 * p2t.z;
        if det < 0. && (t_scaled >= 0. || t_scaled < tmax * det) {
            return None
        } else if det > 0. && (t_scaled <= 0. || t_scaled > tmax * det) {
            return None
        }
        // intersection is now valid, calculate actual coordinates/time of intersect
        let inv_det = 1. / det;
        let b0 = e0 * inv_det;
        let b1 = e1 * inv_det;
        let b2 = e2 * inv_det;
        let t = t_scaled * inv_det;
        if t < SMALL / 10. {
            return None;
        }
        // calculate partial derivatives
        // TODO: Use partial derivatives
        let dpdu: Vector3<f64>;
        let dpdv: Vector3<f64>;
        let uvs: [Vector2<f64>; 3] = self.get_uv(ind);
        let duv02 = uvs[0] - uvs[2];
        let duv12 = uvs[1] - uvs[2];
        let dp02 = p0 - p2;
        let dp12 = p1 - p2;
        let determinant = duv02.x * duv12.y - duv02.y * duv12.x;
        if determinant.abs() < SMALL / 10000. { // special case
            let n: Vector3<f64> = (p2 - p0).cross(&(p1 - p0));
            if n.magnitude_squared() == 0. {
                return None; // degenerate triangle
            }
            let tangents = util::make_coordinate_system(&n);
            dpdu = tangents.0;
            dpdv = tangents.1;
        } else {
            let inv_det = 1. / determinant;
            dpdu = (duv12[1] * dp02 - duv02[1] * dp12) * inv_det;
            dpdv = (-duv12[0] * dp02 + duv02[0] * dp12) * inv_det;
        }
        // use barycentric coordinates to find intersection point
        let p_hit: Point3<f64> = b0 * p0 + b1 * p1.coords + b2 * p2.coords;
        let uv_hit = b0 * uvs[0] + b1 * uvs[1] + b2 * uvs[2];

        let normal: Vector3<f64> = if self.n.len() == 0 { // TODO: differentiate between shader/geometric normal
            dp02.cross(&dp12)
        } else {
            b0 * self.n[ind1] + b1 * self.n[ind2] + b2 * self.n[ind3]
        };
        // fill in record
        let mut record = HitRecord::new(p_hit, uv_hit, -ray.dir, dpdu, dpdv, Vector3::new(0., 0., 0.), 
                                             Vector3::new(0., 0., 0.), t, 0, mat_index);
        record.n = Unit::new_normalize(dp02.cross(&dp12));
        record.shading.n = Unit::new_normalize(normal);
        let mut ss = Unit::new_normalize(dpdu);
        let mut ts = Unit::new_normalize(record.shading.n.cross(&ss));
        if ts.magnitude_squared() > 0f64 {
            ss = Unit::new_normalize(ts.cross(&record.shading.n)); // ensure orthogonality
        } else { // pick arbitrary
            let (temp_ss, temp_ts) = util::make_coordinate_system(&record.shading.n);
            ss = Unit::new_normalize(temp_ss);
            ts = Unit::new_normalize(temp_ts);
        }
        let dndu: Vector3<f64>;
        let dndv: Vector3<f64>;
        if self.n.len() > 0 {
            let determinant = duv02[0] * duv12[1] - duv02[1] * duv12[0];
            let dn1 = self.n[ind1] - self.n[ind3];
            let dn2 = self.n[ind2] - self.n[ind3];
            if determinant.abs() < SMALL / 1000. {
                let dn: Vector3<f64> = (self.n[ind3] - self.n[ind1]).cross(&(self.n[ind2] - self.n[ind1]));
                if dn.magnitude_squared() == 0f64 {
                    dndu = Vector3::new(0., 0., 0.);
                    dndv = Vector3::new(0., 0., 0.);
                } else {
                    let (dnu, dnv) = util::make_coordinate_system(&dn);
                    dndu = dnu;
                    dndv = dnv;
                }
            } else {
                let inv_det = 1f64 / determinant;
                dndu = (duv12[1] * dn1 - duv02[1] * dn2) * inv_det;
                dndv = (-duv12[0] * dn1 + duv02[0] * dn2) * inv_det;
            }
        } else {
            dndu = Vector3::new(0., 0., 0.); // flat
            dndv = Vector3::new(0., 0., 0.);
        }
        record.set_shading_geometry(ss, ts, dndu, dndv, true);
        record.set_front(ray);
        record.uv = uv_hit;
        Some(record)
    }

    pub fn get_uv(&self, ind: usize) -> [Vector2<f64>; 3] {
        if self.uv.len() == 0 {
            return [Vector2::new(0., 0.), Vector2::new(1., 0.), Vector2::new(1., 1.)];
        } else {
            return [self.uv[ind], self.uv[ind + 1], self.uv[ind + 2]];
        }
    }

    #[allow(dead_code)]
    pub fn print_triangle(&self, ind: usize) {
        let ind1 = self.ind[ind];
        let ind2 = self.ind[ind + 1];
        let ind3 = self.ind[ind + 2];
        let p1 = self.p[ind1];
        let p2 = self.p[ind2];
        let p3 = self.p[ind3];
        println!("({:.4}, {:.4}, {:.4}), ({:.4}, {:.4}, {:.4}), ({:.4}, {:.4}, {:.4})", p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, p3.x, p3.y, p3.z);
    }
}

pub struct BoundingBox {
    pub min: Point3<f64>,
    pub max: Point3<f64>,
}

impl BoundingBox {
    pub fn new(min: Point3<f64>, max: Point3<f64>) -> Self { Self { min, max } }
    pub fn intersects(&self, ray: &Ray, mut tmin: f64, mut tmax: f64) -> bool {
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

    pub fn full_intersects(&self, ray: &Ray, tmin: f64, tmax: f64) -> Option<HitRecord> {
        let xy1 = Primitive::new_xy_rect(self.min.x, self.min.y, self.max.x, self.max.y, self.min.z, 0);
        let xy2 = Primitive::new_xy_rect(self.min.x, self.min.y, self.max.x, self.max.y, self.max.z, 0);
        let xz1 = Primitive::new_xz_rect(self.min.x, self.min.z, self.max.x, self.max.z, self.min.y, 0);
        let xz2 = Primitive::new_xz_rect(self.min.x, self.min.z, self.max.x, self.max.z, self.max.y, 0);
        let yz1 = Primitive::new_yz_rect(self.min.y, self.min.z, self.max.y, self.max.z, self.min.x, 0);
        let yz2 = Primitive::new_yz_rect(self.min.y, self.min.z, self.max.y, self.max.z, self.max.x, 0);
        let sides = vec![xy1, xy2, xz1, xz2, yz1, yz2];
        let intersects: Vec<HitRecord> = sides.iter()
                                        .filter_map(|x| Primitive::intersects_obj(x, ray, tmin, tmax))
                                        .filter(|x| x.t >= tmin && x.t <= tmax).collect();
        if intersects.len() == 0 {
            return None;
        }
        let mut time = tmax;
        let mut hit: HitRecord = HitRecord::make_basic(Point3::new(0f64, 0f64, 0f64), 0f64); // random default
        for intersect in intersects {
            if intersect.t <= time {
                if intersect.t >= tmin {
                    time = intersect.t;
                    hit = intersect;
                }
            }
        }
        Some(hit)
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
    pub fn intersects(&self, ray: &Ray, tmin: f64, tmax: f64, depth: i32) -> Option<HitRecord> {
        match self {
            BvhNode::Internal { left, right, bounding_box } => {
                if depth == get_objects().max_bvh {
                    return bounding_box.full_intersects(ray, tmin, tmax);
                }
                if !bounding_box.intersects(ray, tmin, tmax) {
                    return None;
                }
                let left_option = left.intersects(ray, tmin, tmax, depth + 1);
                let right_option = right.intersects(ray, tmin, tmax, depth + 1);

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
                if depth == get_objects().max_bvh {
                    return bounding_box.full_intersects(ray, tmin, tmax);
                }
                if !bounding_box.intersects(ray, tmin, tmax) {
                    return None;
                }
                return Primitive::intersects(*index, ray, tmin, tmax);
            }
            BvhNode::Empty => { return None; }
        }
    }


    #[allow(unused_variables)]
    pub fn new(objects: &Vec<Primitive>, indices: &mut Vec<usize>, start: usize, end: usize, t0: f64, t1: f64) -> BvhNode {
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
            let bbox = Primitive::get_bounding_box(&objects[indices[start]], t0, t1);
            let bbox_copy = BoundingBox::make_copy(bbox);
            return BvhNode::Leaf {index: indices[start], bounding_box: bbox_copy};
        } else if num_obj == 2 {
            if comp(&objects[indices[start]], &objects[indices[start + 1]]) != Ordering::Greater {
                return BvhNode::handle_two(objects, indices, start, start + 1, t0, t1);
            } else {
                return BvhNode::handle_two(objects, indices, start + 1, start, t0, t1);
            }
        } else {
            let slice = &mut indices[start..end];
            slice.sort_by(|a, b| comp(&objects[*a], &objects[*b]));
            let mid = start + num_obj / 2;
            final_left = BvhNode::new(objects, indices, start, mid, t0, t1);
            final_right = BvhNode::new(objects, indices, mid, end, t0, t1);
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

    fn handle_two(objs: &Vec<Primitive>, indices: &mut Vec<usize>, curr: usize, next: usize, t0: f64, t1: f64) -> BvhNode {
        let inner_bb = BoundingBox::make_copy(Primitive::get_bounding_box(&objs[indices[next]], t0, t1));
        let curr_bb = BoundingBox::make_copy(Primitive::get_bounding_box(&objs[indices[curr]], t0, t1));
        let bb = BoundingBox::union(&inner_bb, &curr_bb);
        let inner_right = BvhNode::Leaf {index: indices[curr], bounding_box: curr_bb };
        let inner_left = BvhNode::Leaf { index: indices[next], bounding_box: inner_bb };

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

    #[allow(dead_code)]
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