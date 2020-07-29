use nalgebra::base::{Unit, Vector3, Matrix};
use nalgebra::geometry::{Projective3, Point3};
use std::sync::Arc;

use crate::hittable::BvhNode;
use crate::consts::*;
use crate::util;
use crate::material::materials::{Texture, Material};
use crate::primitive::Primitive;

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

pub fn cast_ray(objs: &Vec<Primitive>, materials: &Vec<Material>, textures: &Vec<Texture>, ray: &Ray, node: &BvhNode, depth: u32) -> Vector3<f64> {
    if depth <= 0 {
        return Vector3::new(0.0, 0.0, 0.0);
    }
    let hit_record = node.intersects(objs, ray, SMALL, INFINITY);
    
    match hit_record {
        Some(record) => {
            let emitted = Material::emit(materials, record.mat_index, textures, record.uv.x, record.uv.y, &record.p);
            let pair = Material::scatter(materials, record.mat_index, textures, ray, &record);
            if pair.is_none() {
                return emitted;
            }
            // if !record.front { // only front-facing objects
            //     return if AMBIENT_LIGHT { util::get_sky(ray) } else { util::get_background(ray) }
            // }
            match pair {
                Some((x, y)) => {
                    let col = cast_ray(objs, materials, textures, &x, node, depth - 1);
                    // if record.mat_index == 1 {
                    //     return col; // + emitted;
                    // }
                    return Vector3::new(col.x * y.x * INV_COL_MAX, col.y * y.y * INV_COL_MAX, col.z * y.z * INV_COL_MAX) + emitted;
                },
                None => Vector3::new(0.0, 0.0, 0.0) // should never happen
            }
        }
        None => if AMBIENT_LIGHT { util::get_sky(ray) } else { util::get_background(ray) }
    }    
}