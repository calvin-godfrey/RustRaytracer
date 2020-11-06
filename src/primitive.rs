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
        light_index: usize,
    },
    Triangle {
        mesh_index: usize,
        ind: usize,
        bounding_box: Option<BoundingBox>,
        mat_index: usize,
        light_index: usize,
    },
    MovingSphere {
        r: f64,
        mat_index: usize,
        t0: f64,
        t1: f64,
        c0: Point3<f64>,
        c1: Point3<f64>,
        bounding_box: Option<BoundingBox>,
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
        Primitive::Sphere {center, r, mat_index, bounding_box, light_index: usize::MAX }
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
        Primitive::MovingSphere { c0, c1, t0, t1, r, mat_index, bounding_box: Some(bounding_box), light_index: usize::MAX }
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
                Primitive::XYRect {x0, y0, x1, y1, k, mat_index, bounding_box: n_box, transform: Some(Arc::clone(&inner_transform)), light_index: usize::MAX }
            }
            None => {
                let bounding_box = Some(BoundingBox::new(Point3::new(x0, y0, k - SMALL), Point3::new(x1, y1, k + SMALL)));
                Primitive::XYRect {x0, y0, x1, y1, k, mat_index, bounding_box, transform: None, light_index: usize::MAX }
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
                Primitive::XZRect {x0, z0, x1, z1, k, mat_index, bounding_box: n_box, transform: Some(Arc::clone(&inner_transform)), light_index: usize::MAX }
            }
            None => {
                let bounding_box = Some(BoundingBox::new(Point3::new(x0, k - SMALL, z0), Point3::new(x1, k + SMALL, z1)));
                Primitive::XZRect {x0, z0, x1, z1, k, mat_index, bounding_box, transform: None, light_index: usize::MAX }
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
                Primitive::YZRect {y0, z0, y1, z1, k, mat_index, bounding_box: n_box, transform: Some(Arc::clone(&inner_transform)), light_index: usize::MAX }
            }
            None => {
                let bounding_box = Some(BoundingBox::new(Point3::new(k - SMALL, y0, z0), Point3::new(k + SMALL, y1, z1)));
                Primitive::YZRect {y0, z0, y1, z1, k, mat_index, bounding_box, transform: None, light_index: usize::MAX }
            }
        }
    }

    pub fn new_flip_face(obj: Box<Primitive>) -> Self {
        Primitive::FlipFace { obj }
    }

    #[allow(unused_variables)]
    pub fn intersects(objs: &Vec<Primitive>, meshes: &Vec<Mesh>, index: usize, ray: &Ray, tmin: f64, tmax: f64) -> Option<HitRecord> {
        let obj = &objs[index];
        let record = match obj {
            Primitive::Sphere { center, r, mat_index, .. } => {sphere_intersect(center, r, *mat_index, ray, tmin, tmax)}
            Primitive::Triangle { mesh_index, ind, mat_index, .. } => {meshes[*mesh_index].intersects_triangle(ray, *ind, *mat_index, tmin, tmax)}
            Primitive::MovingSphere { r, mat_index, t0, t1, c0, c1, .. } => {moving_sphere_intersect(*r, *mat_index, *t0, *t1, c0, c1, ray, tmin, tmax)}
            Primitive::XYRect { x0, y0, x1, y1, k, mat_index, transform, .. } => {xy_rect_intersect(*x0, *y0, *x1, *y1, *k, *mat_index, transform, ray, tmin, tmax)}
            Primitive::XZRect { x0, z0, x1, z1, k, mat_index, transform, .. } => {xz_rect_intersect(*x0, *z0, *x1, *z1, *k, *mat_index, transform, ray, tmin, tmax)}
            Primitive::YZRect { y0, z0, y1, z1, k, mat_index, transform, .. } => {yz_rect_intersect(*y0, *z0, *y1, *z1, *k, *mat_index, transform, ray, tmin, tmax)}
            Primitive::FlipFace { obj } => {
                let hit = Primitive::intersects_obj(obj.as_ref(), meshes, ray, tmin, tmax);
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
        match record {
            Some(mut r) => {
                r.prim_index = index;
                Some(r)
            }
            None => { None }
        }
    }

    pub fn area(&self) -> f64 {
        println!("Error: Unimplemented, line {} in {}.", line!(), file!());
        0.
    }

    pub fn get_light_index(&self) -> usize {
        match self {
            Primitive::Sphere { light_index, .. } => { *light_index },
            Primitive::Triangle { light_index, .. } => { *light_index },
            Primitive::MovingSphere { light_index, .. } => { *light_index },
            Primitive::XYRect { light_index, .. } => { *light_index },
            Primitive::XZRect { light_index, .. } => { *light_index },
            Primitive::YZRect { light_index, .. } => { *light_index },
            Primitive::FlipFace { obj, .. } => { obj.as_ref().get_light_index() },
        }
    }

    pub fn intersects_obj(obj: &Primitive, meshes: &Vec<Mesh>, ray: &Ray, tmin: f64, tmax: f64) -> Option<HitRecord> {
        match obj {
            Primitive::Sphere { center, r, mat_index, .. } => {sphere_intersect(center, r, *mat_index, ray, tmin, tmax)}
            Primitive::Triangle { mesh_index, ind, mat_index, .. } => {meshes[*mesh_index].intersects_triangle(ray, *ind, *mat_index, tmin, tmax)}
            Primitive::MovingSphere { r, mat_index, t0, t1, c0, c1, .. } => {moving_sphere_intersect(*r, *mat_index, *t0, *t1, c0, c1, ray, tmin, tmax)}
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
            Primitive::MovingSphere { bounding_box, .. } => {bounding_box}
            Primitive::XYRect { bounding_box, .. } => {bounding_box}
            Primitive::XZRect { bounding_box, .. } => {bounding_box}
            Primitive::YZRect { bounding_box, .. } => {bounding_box}
            Primitive::FlipFace { obj } => { Primitive::get_bounding_box(obj.as_ref(), time0, time1) }
        }
    }

    pub fn const_pdf(object: &Primitive) -> f64 {
        1f64 / object.area()
    }

    #[allow(unused_variables)]
    pub fn pdf(object: &Primitive, record: &HitRecord, dir: &Vector3<f64>) -> f64 {
        let ray = Ray::new_time(record.p, *dir, record.t);
        todo!("Finish this");
    }

    #[allow(unused_variables)]
    pub fn get_rand_dir(object: &Primitive, origin: &Point3<f64>) -> Vector3<f64> {
        match object {
            Primitive::Sphere { center, r, .. } => {
                (*center + util::rand_cosine_dir().scale(*r)) - *origin
            }
            Primitive::Triangle { .. } => {
                println!("Calling unimplmented funtion, line {} in {}", line!(), file!());
                Vector3::new(0., 0., 0.)
            }
            Primitive::MovingSphere { .. } => {
                println!("Calling unimplmented funtion, line {} in {}", line!(), file!());
                Vector3::new(0., 0., 0.)
            }
            Primitive::XYRect { x0, y0, x1, y1, k, .. } => {
                let x = util::rand_range(*x0, *x1);
                let y = util::rand_range(*y0, *y1);
                Point3::new(x, y, *k) - *origin
            }
            Primitive::XZRect { x0, z0, x1, z1, k, .. } => {
                let x = util::rand_range(*x0, *x1);
                let z = util::rand_range(*z0, *z1);
                Point3::new(x, *k, z) - *origin
            }
            Primitive::YZRect { y0, z0, y1, z1, k, .. } => {
                let z = util::rand_range(*z0, *z1);
                let y = util::rand_range(*y0, *y1);
                Point3::new( *k, y, z) - *origin
            }
            Primitive::FlipFace { obj } => { Primitive::get_rand_dir(obj.as_ref(), origin) }
        }
    }
}

pub fn moving_sphere_center(c0: &Point3<f64>, c1: &Point3<f64>, t0: f64, t1: f64, time: f64) -> Point3<f64> {
    let diff: Vector3<f64> = c1 - c0;
    c0 + (time - t0) / (t1 - t0) * diff
}