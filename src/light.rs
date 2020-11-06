#![allow(dead_code, unused_variables)] // TODO: Remove this
use nalgebra::base::{Unit, Vector3};
use nalgebra::geometry::{Projective3, Point3, Point2};
use crate::hittable::{Visibility, HitRecord};
use crate::geometry::{ONB, Ray};
use crate::consts::*;
use crate::util;
use crate::primitive::Primitive;

fn falloff(light: &Light, w: &Vector3<f64>) -> f64 {
    let wl = Unit::new_normalize(Light::get_inv_transform(light).transform_vector(w));
    let cos_theta = wl.z;
    let (cos_total, cos_falloff) = Light::get_spot_cos(light);
    if cos_theta < cos_total {
        return 0f64;
    } else if cos_theta > cos_falloff {
        return 1f64;
    }
    let delta = (cos_theta - cos_total) / (cos_falloff - cos_total);
    (delta * delta) * (delta * delta)
}

pub enum Light {
    Point { flags: u8, to_world: Projective3<f64>, to_obj: Projective3<f64>, n_samples: u32, p: Point3<f64>, color: Vector3<f64> },
    Spot { flags: u8, to_world: Projective3<f64>, to_obj: Projective3<f64>, n_samples: u32, p: Point3<f64>, color: Vector3<f64>, cos_width: f64, cos_falloff: f64 },
    Distant { flags: u8, to_world: Projective3<f64>, to_obj: Projective3<f64>, n_samples: u32, color: Vector3<f64>, dir: Vector3<f64>, world_center: Point3<f64>, world_radius: f64 },
    Diffuse { flags: u8, to_world: Projective3<f64>, to_obj: Projective3<f64>, n_samples: u32, color: Vector3<f64>, prim_index: usize, area: f64, two_sided: bool },
}

impl Light {
    pub fn get_flags(light: &Light) -> u8 {
        match light {
            Light::Point { flags, .. } => { *flags }
            Light::Spot { flags, .. } => { *flags }
            Light::Distant { flags, .. } => { *flags }
            Light::Diffuse { flags, .. } => { *flags }
        }
    }

    pub fn get_inv_transform(light: &Light) -> &Projective3<f64> {
        match light {
            Light::Point { to_obj, .. } => { to_obj }
            Light::Spot { to_obj, .. } => { to_obj }
            Light::Distant { to_obj, .. } => { to_obj }
            Light::Diffuse { to_obj, .. } => { to_obj }
        }
    }

    pub fn get_spot_cos(light: &Light) -> (f64, f64) {
        match light {
            Light::Spot { cos_width, cos_falloff, .. } => { (*cos_width, *cos_falloff) }
            _ => { (-1f64, -1f64) }
        }
    }

    pub fn is_delta_light(light: &Light) -> bool {
        let flags = Light::get_flags(light);
        return (flags & DELTA_POSITION) > 0 || (flags & DELTA_DIRECTION) > 0;
    }

    /**
    Returns wi, pdf, color; illumination arriving at point from light
    */
    pub fn sample_li<'a, 'b, 'c>(light: &'a Light, record: &'b HitRecord, u: &'c Point2<f64>) -> (Unit<Vector3<f64>>, f64, Vector3<f64>, Visibility<'b>) {
        match light {
            Light::Point { p, color, .. } => {
                let wi = Unit::new_normalize(p - record.p);
                let pdf = 1.;
                let new_record = HitRecord::make_basic(*p, record.t);
                let vis = Visibility::make_visibility(record, new_record);
                (wi, pdf, color.scale(1. / nalgebra::distance_squared(p, &record.p)), vis)
            }
            Light::Spot { p, color, .. } => {
                let wi = Unit::new_normalize(p - record.p);
                let pdf = 1.;
                let new_record = HitRecord::make_basic(*p, record.t);
                let vis = Visibility::make_visibility(record, new_record);
                let color = color.scale(falloff(light, &-wi) / nalgebra::distance_squared(p, &record.p));
                (wi, pdf, color, vis)
            }
            Light::Distant { color, dir, world_radius, .. } => {
                let wi = *dir;
                let pdf = 1.;
                let p_outside = record.p + dir * (2f64 * world_radius);
                let new_record = HitRecord::make_basic(p_outside, record.t);
                let vis = Visibility::make_visibility(record, new_record);
                (Unit::new_normalize(wi), pdf, *color, vis)
            }
            Light::Diffuse { .. } => {todo!("diffuse sample_li")}
        }
    }

    pub fn power(light: &Light) -> Vector3<f64> {
        match light {
            Light::Point {color, .. } => {4. * PI * color }
            Light::Spot { color, cos_width, cos_falloff, .. } => {
                color * 2f64 * PI * (1f64 - 0.5 * (cos_falloff + cos_width))
            }
            Light::Distant { color, world_radius, .. } => {
                color * PI * *world_radius * *world_radius
            }
            Light::Diffuse { two_sided, color, area, .. } => {
                (if *two_sided { 2f64 } else { 1f64 }) * color * *area * PI
            }
        }
    }

    pub fn pdf_li(light: &Light, record: &HitRecord, wi: &Vector3<f64>, prims: &[Primitive]) -> f64 {
        match light {
            Light::Point { .. } => { 0. }
            Light::Spot {.. } => { 0. }
            Light::Distant { .. } => { 0. }
            Light::Diffuse { prim_index, .. } => { Primitive::pdf(&prims[*prim_index], record, wi) }
        }
    }

    /**
    Amount of light leaving from light at direction;
    returns (pdf_pos, pdf_dir, ray, normal, color)
    */
    pub fn sample_le(light: &Light, u1: &Point2<f64>, u2: &Point2<f64>, time: f64) -> (f64, f64, Ray, Vector3<f64>, Vector3<f64>) {
        match light {
            Light::Point { p, color, .. } => {
                let ray = Ray::new_time(*p, util::rand_in_unit_sphere(u1), time);
                let n_light = ray.dir;
                let pdf_pos = 1f64;
                let pdf_dir = util::uniform_sphere_pdf();
                (pdf_pos, pdf_dir, ray, n_light, *color)
            }
            Light::Spot { color, to_world, p, cos_width, .. } => {
                let w = util::uniform_sample_cone(u1, *cos_width);
                let ray = Ray::new_time(*p, to_world.transform_vector(&w), time);
                let n_light = ray.dir;
                let pdf_pos = 1f64;
                let pdf_dir = util::uniform_cone_pdf(*cos_width);
                (pdf_pos, pdf_dir, ray, n_light, *color)
            }
            Light::Distant { color, dir, world_center, world_radius, .. } => {
                let onb = ONB::new_from_vec(dir);
                let cd = util::concentric_sample_disk(u1);
                let p_disk = world_center + (onb.v().scale(cd.x) + onb.w().scale(cd.y)).scale(*world_radius);
                let ray = Ray::new_time(p_disk + dir * *world_radius, -dir, time);
                let n_light = ray.dir;
                let pdf_pos = 1f64 / (PI * *world_radius * *world_radius);
                let pdf_dir = 1f64;
                (pdf_pos, pdf_dir, ray, n_light, *color)
            }
            Light::Diffuse { .. } => {todo!("Diffuse light sample_le")}
        }
    }

    /**
    Returns (pdf_pos, pdf_dir)
    */
    pub fn pdf_le(light: &Light, ray: &Ray, normal: &Vector3<f64>) -> (f64, f64) {
        match light {
            Light::Point { .. } => {
                (0f64, util::uniform_sphere_pdf())
            }
            Light::Spot { .. } => {
                (0f64, util::uniform_sphere_pdf())
            }
            Light::Distant { world_radius, .. } => {
                (1f64 / (PI * *world_radius * *world_radius), 0f64)
            }
            Light::Diffuse { prim_index, two_sided, .. } => { todo!("diffuse pdf_le") }
        }
    }

    pub fn l(light: &Light, record: &HitRecord, w: &Vector3<f64>) -> Vector3<f64> {
        match light {
            Light::Diffuse { two_sided, color, .. } => {
                if record.n.dot(w) > 0f64 || *two_sided {
                    *color
                } else {
                    util::black()
                }
            }
            _ => { util::black() }
        }
    }

    pub fn set_prim_index(&mut self, index: usize) {
        match self {
            Light::Diffuse { prim_index, .. } => { *prim_index = index }
            _ => {}
        }
    }

    pub fn make_point_light(to_world: Projective3<f64>, color: Vector3<f64>, n_samples: u32) -> Self {
        let flags = DELTA_POSITION;
        let transformed_point = to_world.transform_point(&Point3::new(0., 0., 0.));
        Light::Point {flags, to_world, to_obj: to_world.inverse(), n_samples, p: transformed_point, color }
    }

    pub fn make_spot_light(to_world: Projective3<f64>, color: Vector3<f64>, total_width: f64, falloff: f64, n_samples: u32) -> Self {
        let flags = DELTA_POSITION;
        let transformed_point = to_world.transform_point(&Point3::new(0., 0., 0.));
        let cos_width = (total_width * 180f64 * INV_PI).cos();
        let cos_falloff = (falloff * 180f64 * INV_PI).cos();
        Light::Spot {flags, to_world, to_obj: to_world.inverse(), p: transformed_point, color, cos_width, cos_falloff, n_samples }
    }

    pub fn make_distant_light(to_world: Projective3<f64>, color: Vector3<f64>, dir: Vector3<f64>, n_samples: u32) -> Self {
        let flags = DELTA_DIRECTION;
        // TODO: Pick better world radius?
        Light::Distant {flags, to_world, to_obj: to_world.inverse(), color, dir, world_center: Point3::new(0., 0., 0.), world_radius: 1e10, n_samples }
    }

    /**
    Remember to call set_prim_index
    */
    pub fn make_diffuse_light(prim: &Primitive, to_world: Projective3<f64>, color: Vector3<f64>, n_samples: u32, two_sided: bool) -> Self {
        let flags = AREA;
        Light::Diffuse { flags, to_world, to_obj: to_world.inverse(), n_samples, two_sided, color, prim_index: 0, area: prim.area() }
    }
}