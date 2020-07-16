use nalgebra::base::Vector3;
use nalgebra::geometry::{Projective3, Point3};
use std::sync::Arc;
use crate::hittable::{HitRecord, BoundingBox, Mesh};
use crate::geometry::Ray;
use crate::consts::*;
use crate::intersects::*;
use crate::util;

pub enum Primitive {
    Sphere {
        center: Point3<f64>,
        r: f64,
        mat_index: usize,
        bounding_box: Option<BoundingBox>,
    },
    Triangle {
        mesh: Arc<Mesh>,
        ind: usize,
        bounding_box: Option<BoundingBox>,
    },
    MovingSphere {
        r: f64,
        mat_index: usize,
        t0: f64,
        t1: f64,
        c0: Point3<f64>,
        c1: Point3<f64>,
        bounding_box: Option<BoundingBox>,
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
    },
    XZRect {
        x0: f64,
        z0: f64,
        x1: f64,
        z1: f64,
        k: f64,
        mat_index: usize,
        transform:Option< Arc<Projective3<f64>>>,
        bounding_box: Option<BoundingBox>,
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
    },
    FlipFace {
        obj: Box<Primitive>,
    }
}

impl Primitive {
    pub fn new_sphere(center: Point3<f64>, r: f64, mat_index: usize) -> Self {
        let r_vec = Vector3::new(r, r, r);
        let min = center - r_vec;
        let max = center + r_vec;
        let bounding_box = Some(BoundingBox::new(min, max));
        Primitive::Sphere {center, r, mat_index, bounding_box }
    }

    pub fn new_moving_sphere(c0: Point3<f64>, c1: Point3<f64>, t0: f64, t1: f64, r: f64, mat_index: usize) -> Primitive {
        let c0 = moving_sphere_center(&c0, &c1, t0, t1, t0);
        let c1 = moving_sphere_center(&c0, &c1, t0, t1, t1);
        let min0 = Point3::new(c0.x - r, c0.y - r, c0.z - r);
        let max0 = Point3::new(c0.x + r, c0.y + r, c0.z + r);
        let min1 = Point3::new(c1.x - r, c1.y - r, c1.z - r);
        let max1 = Point3::new(c1.x + r, c1.y + r, c1.z + r);
        let bbox1 = BoundingBox::new(min0, max0);
        let bbox2 = BoundingBox::new(min1, max1);
        let bounding_box = BoundingBox::union(&bbox1, &bbox2);
        Primitive::MovingSphere { c0, c1, t0, t1, r, mat_index, bounding_box: Some(bounding_box) }
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
                Primitive::XYRect {x0, y0, x1, y1, k, mat_index, bounding_box: n_box, transform: Some(Arc::clone(&inner_transform))  }
            }
            None => {
                let bounding_box = Some(BoundingBox::new(Point3::new(x0, y0, k - SMALL), Point3::new(x1, y1, k + SMALL)));
                Primitive::XYRect {x0, y0, x1, y1, k, mat_index, bounding_box, transform: None }
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
                Primitive::XZRect {x0, z0, x1, z1, k, mat_index, bounding_box: n_box, transform: Some(Arc::clone(&inner_transform))  }
            }
            None => {
                let bounding_box = Some(BoundingBox::new(Point3::new(x0, k - SMALL, z0), Point3::new(x1, k + SMALL, z1)));
                Primitive::XZRect {x0, z0, x1, z1, k, mat_index, bounding_box, transform: None }
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
                Primitive::YZRect {y0, z0, y1, z1, k, mat_index, bounding_box: n_box, transform: Some(Arc::clone(&inner_transform))  }
            }
            None => {
                let bounding_box = Some(BoundingBox::new(Point3::new(k - SMALL, y0, z0), Point3::new(k + SMALL, y1, z1)));
                Primitive::YZRect {y0, z0, y1, z1, k, mat_index, bounding_box, transform: None }
            }
        }
    }

    pub fn new_flip_face(obj: Box<Primitive>) -> Self {
        Primitive::FlipFace { obj }
    }

    #[allow(unused_variables)]
    pub fn intersects(objs: &Vec<Primitive>, index: usize, ray: &Ray, tmin: f64, tmax: f64) -> Option<HitRecord> {
        let obj = &objs[index];
        match obj {
            Primitive::Sphere { center, r, mat_index, bounding_box } => {sphere_intersect(center, r, *mat_index, ray, tmin, tmax)}
            Primitive::Triangle { mesh, ind, bounding_box } => {mesh.intersects_triangle(ray, *ind, tmin, tmax)}
            Primitive::MovingSphere { r, mat_index, t0, t1, c0, c1, bounding_box } => {moving_sphere_intersect(*r, *mat_index, *t0, *t1, c0, c1, ray, tmin, tmax)}
            Primitive::XYRect { x0, y0, x1, y1, k, mat_index, bounding_box, transform } => {xy_rect_intersect(*x0, *y0, *x1, *y1, *k, *mat_index, transform, ray, tmin, tmax)}
            Primitive::XZRect { x0, z0, x1, z1, k, mat_index, bounding_box, transform } => {xz_rect_intersect(*x0, *z0, *x1, *z1, *k, *mat_index, transform, ray, tmin, tmax)}
            Primitive::YZRect { y0, z0, y1, z1, k, mat_index, bounding_box, transform } => {yz_rect_intersect(*y0, *z0, *y1, *z1, *k, *mat_index, transform, ray, tmin, tmax)}
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
        }
    }

    #[allow(unused_variables)]
    pub fn intersects_obj(obj: &Primitive, ray: &Ray, tmin: f64, tmax: f64) -> Option<HitRecord> {
        match obj {
            Primitive::Sphere { center, r, mat_index, bounding_box } => {sphere_intersect(center, r, *mat_index, ray, tmin, tmax)}
            Primitive::Triangle { mesh, ind, bounding_box } => {mesh.intersects_triangle(ray, *ind, tmin, tmax)}
            Primitive::MovingSphere { r, mat_index, t0, t1, c0, c1, bounding_box } => {moving_sphere_intersect(*r, *mat_index, *t0, *t1, c0, c1, ray, tmin, tmax)}
            Primitive::XYRect { x0, y0, x1, y1, k, mat_index, bounding_box, transform } => {xy_rect_intersect(*x0, *y0, *x1, *y1, *k, *mat_index, transform, ray, tmin, tmax)}
            Primitive::XZRect { x0, z0, x1, z1, k, mat_index, bounding_box, transform } => {xz_rect_intersect(*x0, *z0, *x1, *z1, *k, *mat_index, transform, ray, tmin, tmax)}
            Primitive::YZRect { y0, z0, y1, z1, k, mat_index, bounding_box, transform } => {yz_rect_intersect(*y0, *z0, *y1, *z1, *k, *mat_index, transform, ray, tmin, tmax)}
            Primitive::FlipFace { obj } => { None } // FlipFace inside a flip face, should never happen
        }
    }

    #[allow(unused_variables)]
    pub fn get_bounding_box(obj: &Primitive, time0: f64, time1: f64) -> &Option<BoundingBox> {
        match obj {
            Primitive::Sphere { center, r, mat_index, bounding_box } => { bounding_box }
            Primitive::Triangle { mesh, ind, bounding_box } => {bounding_box}
            Primitive::MovingSphere { r, mat_index, t0, t1, c0, c1, bounding_box } => {bounding_box}
            Primitive::XYRect { x0, y0, x1, y1, k, mat_index, bounding_box, transform } => {bounding_box}
            Primitive::XZRect { x0, z0, x1, z1, k, mat_index, bounding_box, transform } => {bounding_box}
            Primitive::YZRect { y0, z0, y1, z1, k, mat_index, bounding_box, transform } => {bounding_box}
            Primitive::FlipFace { obj } => { Primitive::get_bounding_box(obj.as_ref(), time0, time1) }
        }
    }
}

pub fn moving_sphere_center(c0: &Point3<f64>, c1: &Point3<f64>, t0: f64, t1: f64, time: f64) -> Point3<f64> {
    let diff: Vector3<f64> = c1 - c0;
    c0 + (time - t0) / (t1 - t0) * diff
}