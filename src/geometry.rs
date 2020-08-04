use nalgebra::base::{Unit, Vector3, Matrix};
use nalgebra::geometry::{Projective3, Point3};
use std::sync::Arc;

use crate::hittable::BvhNode;
use crate::consts::*;
use crate::util;
use crate::material::materials::{Texture, Material};
use crate::primitive::Primitive;
use crate::pdf::PDF;

#[derive(Copy, Clone)]
pub struct ONB {
    axis: [Unit<Vector3<f64>>; 3],
}

impl ONB {
    pub fn u(&self) -> Unit<Vector3<f64>> { self.axis[0] }
    pub fn v(&self) -> Unit<Vector3<f64>> { self.axis[1] }
    pub fn w(&self) -> Unit<Vector3<f64>> { self.axis[2] }

    pub fn get_local(&self, a: f64, b: f64, c: f64) -> Vector3<f64> {
        self.u().scale(a) + self.v().scale(b) + self.w().scale(c)
    }

    pub fn get_local_vec(&self, a: &Vector3<f64>) -> Vector3<f64> {
        self.u().scale(a.x) + self.v().scale(a.y) + self.w().scale(a.z)
    }

    pub fn new_from_vec(n: &Vector3<f64>) -> Self {
        let w = Unit::new_normalize(*n);
        let a = if w.x.abs() > 0.9 { Unit::new_normalize(Vector3::new(0., 1., 0.)) } else { Unit::new_normalize(Vector3::new(1., 0., 0.)) };
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
    t0: f64, // shutter open
    t1: f64, // shutter close
    u: Unit<Vector3<f64>>, // horizontal
    v: Unit<Vector3<f64>>, // vertical
    #[allow(dead_code)]
    w: Unit<Vector3<f64>>, // direction of sight
}

impl Camera {
    pub fn new(from: Point3<f64>, to: Point3<f64>, up: Vector3<f64>, aspect_ratio: f64, vfov: f64, aperture: f64, focus_dist: f64) -> Self {
        Camera::new_motion_blur(from, to, up, aspect_ratio, vfov, aperture, focus_dist, 0., 0.)
    }

    pub fn new_motion_blur(from: Point3<f64>, to: Point3<f64>, up: Vector3<f64>, aspect_ratio: f64, vfov: f64, aperture: f64, focus_dist: f64, t0: f64, t1: f64) -> Self {
        let w: Unit<Vector3<f64>> = Unit::new_normalize(from - to);
        let u: Unit<Vector3<f64>> = Unit::new_normalize(Matrix::cross(&up, &w));
        let v: Unit<Vector3<f64>> = Unit::new_normalize(Matrix::cross(&w, &u));

        let theta = vfov * PI / 180.;
        let h = (theta / 2.).tan();
        let viewport_height = 2. * h;
        let viewport_width = viewport_height * aspect_ratio;

        let origin: Point3<f64> = from;
        let horizontal_offset: Vector3<f64> = u.scale(viewport_width * focus_dist);
        let vertical_offset: Vector3<f64> = v.scale(viewport_height * focus_dist);
        // this is the point in world space that represents the bottom left corner of the plane that is being projected onto
        let upper_left_corner: Point3<f64> = origin -
                                             horizontal_offset.scale(0.5) +
                                             vertical_offset.scale(0.5) -
                                             w.as_ref().scale(focus_dist);

        let lens_radius = aperture / 2.;
        
        return Self {origin, upper_left_corner, horizontal_offset, vertical_offset, lens_radius, u, v, w, t0, t1};
    }

    pub fn get_ray(&self, u: f64, v: f64) -> Ray {
        let in_disk: Vector3<f64> = util::rand_in_disk().scale(self.lens_radius);
        let offset = self.u.scale(in_disk.x) + self.v.scale(in_disk.y);

        let to: Point3<f64> = self.upper_left_corner + self.horizontal_offset.scale(u) - self.vertical_offset.scale(v);
        let dir: Vector3<f64> = to - self.origin;
        // have to subtract offset from the direction so that it points back to where it was originally supposed to
        return Ray::new_time(self.origin + offset, dir - offset, util::rand_range(self.t0, self.t1));
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
    pub fn new(origin: Point3<f64>, dir: Vector3<f64>) -> Self { Ray::new_time(origin, dir, 0.) }
    pub fn new_time(origin: Point3<f64>, dir: Vector3<f64>, time: f64) -> Self { Self { origin, dir, time } }

    pub fn at(&self, t: f64) -> Point3<f64> {
        self.origin + self.dir.scale(t)
    }

    pub fn transform(&self, trans: &Arc<Projective3<f64>>) -> Self {
        let new_dir = trans.inverse_transform_vector(&self.dir);
        let new_origin = trans.inverse_transform_point(&self.origin);
        Ray::new_time(new_origin, new_dir, self.time)
    }
}

pub fn cast_ray(objs: &Vec<Primitive>, lights: &Vec<usize>, materials: &Vec<Material>, textures: &Vec<Texture>, ray: &Ray, node: &BvhNode, depth: u32) -> Vector3<f64> {
    if depth <= 0 {
        return Vector3::new(0.0, 0.0, 0.0);
    }
    let hit_record = node.intersects(objs, ray, SMALL, INFINITY);
    
    match hit_record {
        Some(record) => {
            let emitted = Material::emit(materials, record.mat_index, textures, &record);
            let pair = Material::scatter(materials, record.mat_index, textures, ray, &record);
            if pair.is_none() {
                return emitted;
            }
            // if !record.front { // only front-facing objects
            //     return if AMBIENT_LIGHT { util::get_sky(ray) } else { util::get_background(ray) }
            // }
            let cos_pdf = PDF::new_cos(&record.n);
            let pdf = if lights.len() == 0 {
                cos_pdf
            } else {
                let index = util::rand_int(0, lights.len() as i32 - 1);
                let light_pdf = PDF::new_hittable(lights[index as usize], record.p);
                PDF::new_mixture(cos_pdf, light_pdf)
            };

            match pair {
                Some((new_ray, albedo, pdf_val)) => {
                    let new_ray = Ray::new_time(record.p, PDF::generate(&pdf, &objs), ray.time);
                    let pdf_val = PDF::value(&pdf, &objs, &new_ray.dir);
                    if pdf_val == 0. {
                        return emitted;
                    }
                    let col = cast_ray(objs, lights, materials, textures, &new_ray, node, depth - 1);
                    let albedo_scale = Material::scattering_pdf(materials, record.mat_index, textures, ray, &record, &new_ray);
                    return col.scale(1. / pdf_val).component_mul(&albedo.scale(albedo_scale)) + emitted;
                },
                None => Vector3::new(0.0, 0.0, 0.0) // should never happen
            }
        }
        None => if AMBIENT_LIGHT { util::get_sky(ray) } else { util::get_background(ray) }
    }    
}