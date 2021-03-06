use bumpalo::Bump;
use nalgebra::base::{Matrix, Unit, Vector3};
use nalgebra::geometry::{Point2, Point3, Projective3};
use std::sync::Arc;

use crate::consts::*;
use crate::hittable::{BvhNode, HitRecord, Mesh};
use crate::light::Light;
use crate::material::materials::{Material, Texture};
use crate::primitive::Primitive;
use crate::util;

pub struct Objects {
    pub meshes: Vec<Mesh>,
    pub objs: Vec<Primitive>,
    pub lights: Vec<Light>,
    pub materials: Vec<Material>,
    pub textures: Vec<Texture>,
    pub node: BvhNode,
    pub max_bvh: i32,
}
static mut OBJECTS: Objects = Objects {
    meshes: Vec::new(),
    objs: Vec::new(),
    lights: Vec::new(),
    materials: Vec::new(),
    textures: Vec::new(),
    node: BvhNode::Empty,
    max_bvh: std::i32::MAX,
};

pub fn get_objects() -> &'static Objects {
    unsafe { &OBJECTS }
}

pub fn get_objects_mut() -> &'static mut Objects {
    // this is technically safe because it's
    // only ever used in the single-thread stage of the program
    unsafe { &mut OBJECTS }
}

pub fn clear_objects() {
    // this is only ever used to switch between scenes in the UI
    unsafe {
        OBJECTS = Objects {
            meshes: Vec::new(),
            objs: Vec::new(),
            lights: Vec::new(),
            materials: Vec::new(),
            textures: Vec::new(),
            node: BvhNode::Empty,
            max_bvh: std::i32::MAX,
        };
    }
}

#[derive(Copy, Clone)]
pub struct ONB {
    axis: [Unit<Vector3<f64>>; 3],
}

#[allow(dead_code)]
impl ONB {
    pub fn u(&self) -> Unit<Vector3<f64>> {
        self.axis[0]
    }
    pub fn v(&self) -> Unit<Vector3<f64>> {
        self.axis[1]
    }
    pub fn w(&self) -> Unit<Vector3<f64>> {
        self.axis[2]
    }

    pub fn get_local(&self, a: f64, b: f64, c: f64) -> Vector3<f64> {
        self.u().scale(a) + self.v().scale(b) + self.w().scale(c)
    }

    pub fn get_local_vec(&self, a: &Vector3<f64>) -> Vector3<f64> {
        self.u().scale(a.x) + self.v().scale(a.y) + self.w().scale(a.z)
    }

    pub fn new_from_vec(n: &Vector3<f64>) -> Self {
        let w = Unit::new_normalize(*n);
        let a = if w.x.abs() > 0.9 {
            Unit::new_normalize(Vector3::new(0., 1., 0.))
        } else {
            Unit::new_normalize(Vector3::new(1., 0., 0.))
        };
        let b = Unit::new_normalize(w.cross(&a));
        let axis: [Unit<Vector3<f64>>; 3] = [b, a, w];
        Self { axis }
    }
}

#[derive(Copy, Clone)]
pub struct Camera {
    origin: Point3<f64>,
    upper_left_corner: Point3<f64>,
    horizontal_offset: Vector3<f64>,
    vertical_offset: Vector3<f64>,
    lens_radius: f64,
    t0: f64,               // shutter open
    t1: f64,               // shutter close
    u: Unit<Vector3<f64>>, // horizontal
    v: Unit<Vector3<f64>>, // vertical
    #[allow(dead_code)]
    w: Unit<Vector3<f64>>, // direction of sight
}

impl Camera {
    pub fn new(
        from: Point3<f64>,
        to: Point3<f64>,
        up: Vector3<f64>,
        aspect_ratio: f64,
        vfov: f64,
        aperture: f64,
        focus_dist: f64,
    ) -> Self {
        Camera::new_motion_blur(
            from,
            to,
            up,
            aspect_ratio,
            vfov,
            aperture,
            focus_dist,
            0.,
            0.,
        )
    }

    pub fn new_motion_blur(
        from: Point3<f64>,
        to: Point3<f64>,
        up: Vector3<f64>,
        aspect_ratio: f64,
        vfov: f64,
        aperture: f64,
        focus_dist: f64,
        t0: f64,
        t1: f64,
    ) -> Self {
        let w: Unit<Vector3<f64>> = Unit::new_normalize(to - from);
        let u: Unit<Vector3<f64>> = -Unit::new_normalize(Matrix::cross(&up, &w));
        let v: Unit<Vector3<f64>> = -Unit::new_normalize(Matrix::cross(&w, &u));

        let theta = vfov * PI / 180.;
        let h = (theta / 2.).tan();
        let viewport_height = 2. * h;
        let viewport_width = viewport_height * aspect_ratio;

        let origin: Point3<f64> = from;
        let horizontal_offset: Vector3<f64> = u.scale(viewport_width * focus_dist);
        let vertical_offset: Vector3<f64> = v.scale(viewport_height * focus_dist);
        // this is the point in world space that represents the bottom left corner of the plane that is being projected onto
        let upper_left_corner: Point3<f64> = origin - horizontal_offset.scale(0.5)
            + vertical_offset.scale(0.5)
            + w.as_ref().scale(focus_dist);

        let lens_radius = aperture / 2.;

        return Self {
            origin,
            upper_left_corner,
            horizontal_offset,
            vertical_offset,
            lens_radius,
            u,
            v,
            w,
            t0,
            t1,
        };
    }

    pub fn get_ray(&self, u: f64, v: f64) -> Ray {
        let in_disk: Vector3<f64> = util::rand_in_disk().scale(self.lens_radius);
        let offset = self.u.scale(in_disk.x) + self.v.scale(in_disk.y);

        let to: Point3<f64> = self.upper_left_corner + self.horizontal_offset.scale(u)
            - self.vertical_offset.scale(v);
        let dir: Vector3<f64> = to - self.origin;
        // have to subtract offset from the direction so that it points back to where it was originally supposed to
        return Ray::new_time(
            self.origin + offset,
            dir - offset,
            util::rand_range(self.t0, self.t1),
        );
    }

    pub fn translate(&self, tx: f64, ty: f64, tz: f64) -> Camera {
        let fx = self.w.scale(tx);
        let fy = self.u.scale(ty);
        let fz = self.v.scale(tz);
        Camera {
            origin: self.origin + fx + fy + fz,
            upper_left_corner: self.upper_left_corner + 2f64 * fx + fy + fz,
            horizontal_offset: self.horizontal_offset,
            vertical_offset: self.vertical_offset,
            lens_radius: self.lens_radius,
            u: self.u,
            v: self.v,
            w: self.w,
            t0: self.t0,
            t1: self.t1,
        }
    }
}

#[derive(Copy, Clone)]
pub struct Ray {
    pub origin: Point3<f64>,
    pub dir: Vector3<f64>,
    pub time: f64,
}

impl Ray {
    #[allow(dead_code)]
    pub fn new(origin: Point3<f64>, dir: Vector3<f64>) -> Self {
        Ray::new_time(origin, dir, 0.)
    }
    pub fn new_time(origin: Point3<f64>, dir: Vector3<f64>, time: f64) -> Self {
        Self { origin, dir, time }
    }

    pub fn at(&self, t: f64) -> Point3<f64> {
        self.origin + self.dir.scale(t)
    }

    pub fn transform(&self, trans: &Arc<Projective3<f64>>) -> Self {
        let new_dir = trans.inverse_transform_vector(&self.dir);
        let new_origin = trans.inverse_transform_point(&self.origin);
        Ray::new_time(new_origin, new_dir, self.time)
    }
}

pub fn get_intersection(ray: &Ray) -> Option<HitRecord> {
    get_objects().node.intersects(ray, SMALL, INFINITY, 0)
}

pub fn cast_ray(ray: &Ray, depth: u32) -> Vector3<f64> {
    if depth <= 0 {
        return Vector3::new(0.0, 0.0, 0.0);
    }
    let hit_record = get_intersection(ray);
    let objs = get_objects();

    match hit_record {
        Some(mut record) => {
            if objs.objs[record.prim_index].get_light_index() != std::usize::MAX {
                return record.le(&-ray.dir);
            }
            let arena = Bump::new();
            // TODO: Use right mode
            Material::compute_scattering(&mut record, &arena, RADIANCE, true);
            let sample = Point2::new(util::rand(), util::rand());
            let (color, new_dir, pdf, _) = record.bsdf.sample_f(&-ray.dir, &sample, BSDF_ALL);
            if pdf == 0. {
                return util::black();
            }
            let new_ray = Ray::new_time(record.p, new_dir, ray.time);
            let incoming = cast_ray(&new_ray, depth - 1);
            let ans = incoming
                .component_mul(&color.scale(1f64 / pdf))
                .scale(ray.dir.dot(&record.shading.n).abs());
            ans
        }
        None => {
            let mut l = util::black();
            for light in &objs.lights {
                l = l + light.le(ray) / (4f64 * PI);
            }
            l
        }
    }
}
