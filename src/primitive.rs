use nalgebra::base::{Unit, Vector3};
use nalgebra::geometry::{Projective3, Point3, Point2};
use std::sync::Arc;
use crate::hittable::{HitRecord, BoundingBox};
use crate::geometry::{get_objects, Ray};
use crate::consts::*;
use crate::intersects::*;
use crate::util;

pub enum Primitive {
    Sphere {
        center: Point3<f64>,
        r: f64,
        mat_index: usize,
        bounding_box: Option<BoundingBox>,
        light_index: usize,
    },
    Triangle {
        mesh_index: usize,
        ind: usize,
        bounding_box: Option<BoundingBox>,
        mat_index: usize,
        light_index: usize,
    },
    XYRect {
        x0: f64,
        y0: f64,
        x1: f64,
        y1: f64,
        k: f64,
        mat_index: usize,
        transform: Option<Arc<Projective3<f64>>>,
        bounding_box: Option<BoundingBox>,
        light_index: usize,
    },
    XZRect {
        x0: f64,
        z0: f64,
        x1: f64,
        z1: f64,
        k: f64,
        mat_index: usize,
        transform:Option<Arc<Projective3<f64>>>,
        bounding_box: Option<BoundingBox>,
        light_index: usize,
    },
    YZRect {
        y0: f64,
        z0: f64,
        y1: f64,
        z1: f64,
        k: f64,
        mat_index: usize,
        transform: Option<Arc<Projective3<f64>>>,
        bounding_box: Option<BoundingBox>,
        light_index: usize,
    },
    FlipFace {
        obj: Box<Primitive>,
    },
}

impl Primitive {
    pub fn new_sphere(center: Point3<f64>, r: f64, mat_index: usize) -> Self {
        let r_vec = Vector3::new(r, r, r);
        let min = center - r_vec;
        let max = center + r_vec;
        let bounding_box = Some(BoundingBox::new(min, max));
        Primitive::Sphere {center, r, mat_index, bounding_box, light_index: std::usize::MAX }
    }

    pub fn new_xy_rect(x0: f64, y0: f64, x1: f64, y1: f64, k: f64, mat_index: usize) -> Primitive {
        Primitive::new_xy_rect_transform(x0, y0, x1, y1, k, mat_index, None)
    }

    pub fn new_xy_rect_transform(x0: f64, y0: f64, x1: f64, y1: f64, k: f64, mat_index: usize, transform: Option<Arc<Projective3<f64>>>) -> Primitive {
        match transform {
            Some(inner_transform) => {
                let p1 = Point3::new(x0, y0, k - SMALL);
                let p2 = Point3::new(x1, y1, k + SMALL);
                let bounding_box = BoundingBox::new(p1, p2);
                let n_box = Some(util::get_new_box(bounding_box, &inner_transform));
                Primitive::XYRect {x0, y0, x1, y1, k, mat_index, bounding_box: n_box, transform: Some(Arc::clone(&inner_transform)), light_index: std::usize::MAX }
            }
            None => {
                let bounding_box = Some(BoundingBox::new(Point3::new(x0, y0, k - SMALL), Point3::new(x1, y1, k + SMALL)));
                Primitive::XYRect {x0, y0, x1, y1, k, mat_index, bounding_box, transform: None, light_index: std::usize::MAX }
            }
        }
    }

    pub fn new_xz_rect(x0: f64, z0: f64, x1: f64, z1: f64, k: f64, mat_index: usize) -> Primitive {
        Primitive::new_xz_rect_transform(x0, z0, x1, z1, k, mat_index, None)
    }

    pub fn new_xz_rect_transform(x0: f64, z0: f64, x1: f64, z1: f64, k: f64, mat_index: usize, transform: Option<Arc<Projective3<f64>>>) -> Primitive {
        match transform {
            Some(inner_transform) => {
                let p1 = Point3::new(x0, k - SMALL, z0);
                let p2 = Point3::new(x1, k + SMALL, z1);
                let bounding_box = BoundingBox::new(p1, p2);
                let n_box = Some(util::get_new_box(bounding_box, &inner_transform));
                Primitive::XZRect {x0, z0, x1, z1, k, mat_index, bounding_box: n_box, transform: Some(Arc::clone(&inner_transform)), light_index: std::usize::MAX }
            }
            None => {
                let bounding_box = Some(BoundingBox::new(Point3::new(x0, k - SMALL, z0), Point3::new(x1, k + SMALL, z1)));
                Primitive::XZRect {x0, z0, x1, z1, k, mat_index, bounding_box, transform: None, light_index: std::usize::MAX }
            }
        }
    }

    pub fn new_yz_rect(y0: f64, z0: f64, y1: f64, z1: f64, k: f64, mat_index: usize) -> Primitive {
        Primitive::new_yz_rect_transform(y0, z0, y1, z1, k, mat_index, None)
    }

    pub fn new_yz_rect_transform(y0: f64, z0: f64, y1: f64, z1: f64, k: f64, mat_index: usize, transform: Option<Arc<Projective3<f64>>>) -> Primitive {
        match transform {
            Some(inner_transform) => {
                let p1 = Point3::new(k - SMALL, y0, z0);
                let p2 = Point3::new(k + SMALL, y1, z1);
                let bounding_box = BoundingBox::new(p1, p2);
                let n_box = Some(util::get_new_box(bounding_box, &inner_transform));
                Primitive::YZRect {y0, z0, y1, z1, k, mat_index, bounding_box: n_box, transform: Some(Arc::clone(&inner_transform)), light_index: std::usize::MAX }
            }
            None => {
                let bounding_box = Some(BoundingBox::new(Point3::new(k - SMALL, y0, z0), Point3::new(k + SMALL, y1, z1)));
                Primitive::YZRect {y0, z0, y1, z1, k, mat_index, bounding_box, transform: None, light_index: std::usize::MAX }
            }
        }
    }

    pub fn new_flip_face(obj: Box<Primitive>) -> Self {
        Primitive::FlipFace { obj }
    }

    #[allow(unused_variables)]
    pub fn intersects(index: usize, ray: &Ray, tmin: f64, tmax: f64) -> Option<HitRecord> {
        let all_objects = get_objects();
        let obj = &all_objects.objs[index];
        let record = match obj {
            Primitive::Sphere { center, r, mat_index, .. } => {sphere_intersect(center, r, *mat_index, ray, tmin, tmax)}
            Primitive::Triangle { mesh_index, ind, mat_index, .. } => {all_objects.meshes[*mesh_index].intersects_triangle(ray, *ind, *mat_index, tmin, tmax)}
            Primitive::XYRect { x0, y0, x1, y1, k, mat_index, transform, .. } => {xy_rect_intersect(*x0, *y0, *x1, *y1, *k, *mat_index, transform, ray, tmin, tmax)}
            Primitive::XZRect { x0, z0, x1, z1, k, mat_index, transform, .. } => {xz_rect_intersect(*x0, *z0, *x1, *z1, *k, *mat_index, transform, ray, tmin, tmax)}
            Primitive::YZRect { y0, z0, y1, z1, k, mat_index, transform, .. } => {yz_rect_intersect(*y0, *z0, *y1, *z1, *k, *mat_index, transform, ray, tmin, tmax)}
            Primitive::FlipFace { obj } => {
                let hit = Primitive::intersects_obj(obj.as_ref(), ray, tmin, tmax);
                // get the hitrecord of inner, then flip the front-ness
                if hit.is_none() {
                    None
                } else {
                    let mut record = hit.unwrap();
                    record.front = !record.front;
                    Some(record)
                }
            }
        };
        record.map(|mut r| { r.prim_index = index; r })
    }

    pub fn area(&self) -> f64 {
        match self {
            Primitive::Sphere { r, .. } => { return 2f64 * PI * r }
            Primitive::Triangle { mesh_index, ind, .. } => {
                let objs = get_objects();
                let mesh = &objs.meshes[*mesh_index];
                let p0 = mesh.p[mesh.ind[*ind]];
                let p1 = mesh.p[mesh.ind[*ind + 1]];
                let p2 = mesh.p[mesh.ind[*ind + 2]];
                let v1: Vector3<f64> = p1 - p0;
                let v2: Vector3<f64> = p2 - p0;
                return 0.5 * v1.cross(&v2).magnitude();
                
            }
            Primitive::XYRect { x0, y0, x1, y1, .. } => { return (x1 - x0) * (y1 - y0) }
            Primitive::XZRect { x0, z0, x1, z1, .. } => { return (x1 - x0) * (z1 - z0) }
            Primitive::YZRect { y0, z0, y1, z1, .. } => { return (y1 - y0) * (z1 - z0) }
            Primitive::FlipFace { obj } => { return obj.as_ref().area() }
        }
    }

    pub fn get_light_index(&self) -> usize {
        match self {
            Primitive::Sphere { light_index, .. } => { *light_index },
            Primitive::Triangle { light_index, .. } => { *light_index },
            Primitive::XYRect { light_index, .. } => { *light_index },
            Primitive::XZRect { light_index, .. } => { *light_index },
            Primitive::YZRect { light_index, .. } => { *light_index },
            Primitive::FlipFace { obj, .. } => { obj.as_ref().get_light_index() },
        }
    }

    pub fn intersects_obj(obj: &Primitive, ray: &Ray, tmin: f64, tmax: f64) -> Option<HitRecord> {
        match obj {
            Primitive::Sphere { center, r, mat_index, .. } => {sphere_intersect(center, r, *mat_index, ray, tmin, tmax)}
            Primitive::Triangle { mesh_index, ind, mat_index, .. } => {get_objects().meshes[*mesh_index].intersects_triangle(ray, *ind, *mat_index, tmin, tmax)}
            Primitive::XYRect { x0, y0, x1, y1, k, mat_index, transform, .. } => {xy_rect_intersect(*x0, *y0, *x1, *y1, *k, *mat_index, transform, ray, tmin, tmax)}
            Primitive::XZRect { x0, z0, x1, z1, k, mat_index, transform, .. } => {xz_rect_intersect(*x0, *z0, *x1, *z1, *k, *mat_index, transform, ray, tmin, tmax)}
            Primitive::YZRect { y0, z0, y1, z1, k, mat_index, transform, .. } => {yz_rect_intersect(*y0, *z0, *y1, *z1, *k, *mat_index, transform, ray, tmin, tmax)}
            Primitive::FlipFace { .. } => { None } // FlipFace inside a flip face, should never happen
        }
    }

    pub fn get_bounding_box(obj: &Primitive, time0: f64, time1: f64) -> &Option<BoundingBox> {
        match obj {
            Primitive::Sphere { bounding_box, .. } => { bounding_box }
            Primitive::Triangle { bounding_box, .. } => {bounding_box}
            Primitive::XYRect { bounding_box, .. } => {bounding_box}
            Primitive::XZRect { bounding_box, .. } => {bounding_box}
            Primitive::YZRect { bounding_box, .. } => {bounding_box}
            Primitive::FlipFace { obj } => { Primitive::get_bounding_box(obj.as_ref(), time0, time1) }
        }
    }

    pub fn const_pdf(&self) -> f64 {
        1f64 / self.area()
    }

    /**
    Returns hitrecord, pdf, outgoing vector direction
    */
    pub fn sample(&self, in_record: &HitRecord, u: &Point2<f64>) -> (HitRecord, f64, Vector3<f64>) {
        // TODO: Sample based on visible area
        let (record, mut pdf) = self.sample_area(u);
        let wi: Vector3<f64> = record.p - in_record.p;
        if wi.magnitude_squared() == 0f64 {
            pdf = 0f64;
        } else {
            let wi = Unit::new_normalize(wi);
            pdf = pdf * (in_record.p - record.p).magnitude_squared() / (record.n.dot(&-wi).abs());
        }
        (record, pdf, wi)
    }

    /**
    pdf with respect to solid angle
    */
    #[allow(unused_variables)]
    pub fn pdf(&self, record: &HitRecord, dir: &Vector3<f64>) -> f64 {
        self.const_pdf()
        // let ray = record.spawn_ray(dir);
        // let new_record = Primitive::intersects_obj(self, &ray, record.t, INFINITY);
        // if new_record.is_none() {
        //     return 0f64
        // }
        // let new_record = new_record.unwrap();
        // let dist: Vector3<f64> = record.p - new_record.p;
        // dist.magnitude_squared() / (self.area() * new_record.n.dot(&-dir).abs())
    }

    /**
    returns HitRecord, pdf
    */
    pub fn sample_area(&self, u: &Point2<f64>) -> (HitRecord, f64) {
        // TODO: Use the better sample method for spheres
        match self {
            Primitive::Sphere { r, .. } => {
                let p_obj = Point3::new(0f64, 0f64, 0f64) + *r * util::uniform_sample_sphere(u);
                let vp = p_obj - Point3::new(0f64, 0f64, 0f64);
                let record = HitRecord::make_normal(p_obj, Unit::new_normalize(vp));
                (record, 1f64 / self.area())
            }
            Primitive::Triangle { mesh_index, ind, bounding_box, mat_index, light_index } => {
                let meshes = &get_objects().meshes;
                let mesh = &meshes[*mesh_index];
                let b = util::uniform_sample_triangle(u);
                let p0 = mesh.p[mesh.ind[*ind]];
                let p1 = mesh.p[mesh.ind[*ind + 1]];
                let p2 = mesh.p[mesh.ind[*ind + 2]];
                let p: Point3<f64> = Point3::new(0f64, 0f64, 0f64) + (b[0] * p0.coords + b[1] * p1.coords + (1f64 - b[0] - b[1]) * p2.coords);
                let mut record = HitRecord::make_normal(p, Unit::new_normalize(Vector3::new(1f64, 1f64, 1f64)));
                if mesh.n.len() > 0 {
                    let normal: Vector3<f64> = b[0] * mesh.n[mesh.ind[*ind]] + b[1] * mesh.n[mesh.ind[*ind + 1]] + (1f64 - b[0] - b[1]) * mesh.n[mesh.ind[*ind + 2]];
                    record.n = Unit::new_normalize(normal);
                } else {
                    record.n = Unit::new_normalize((p1 - p0).cross(&(p2 - p0)));
                }
                (record, 1f64 / self.area())
            }
            Primitive::XYRect { x0, y0, x1, y1, k, .. } => {
                let n = Unit::new_normalize(Vector3::new(0f64, 0f64, 1f64));
                let p = Point3::new(x0 + u[0] * (x1 - x0), y0 + u[1] * (y1 - y0), *k);
                (HitRecord::make_normal(p, n), 1f64/self.area())
            }
            Primitive::XZRect { x0, z0, x1, z1, k, .. } => {
                let n = Unit::new_normalize(Vector3::new(0f64, 1f64, 0f64));
                let p = Point3::new(x0 + u[0] * (x1 - x0), *k, z0 + u[1] * (z1 - z0));
                (HitRecord::make_normal(p, n), 1f64/self.area())
            }
            Primitive::YZRect { y0, z0, y1, z1, k, .. } => {
                let n = Unit::new_normalize(Vector3::new(1f64, 0f64, 0f64));
                let p = Point3::new(*k, y0 + u[0] * (y1 - y0), z0 + u[1] * (z1 - z0));
                (HitRecord::make_normal(p, n), 1f64/self.area())
            }
            Primitive::FlipFace { obj } => { 
                let (mut record, pdf) = obj.as_ref().sample_area(u);
                record.n = -record.n;
                (record, pdf)
             }
        }
    }
}

pub fn moving_sphere_center(c0: &Point3<f64>, c1: &Point3<f64>, t0: f64, t1: f64, time: f64) -> Point3<f64> {
    let diff: Vector3<f64> = c1 - c0;
    c0 + (time - t0) / (t1 - t0) * diff
}