#![allow(dead_code, unused_variables)] // TODO: Remove this
use crate::consts::*;
use crate::geometry::{get_objects, Ray, ONB};
use crate::material::materials::Texture;
use crate::primitive::Primitive;
use crate::util;
use crate::{
    distribution::Distribution2D,
    hittable::{HitRecord, Visibility},
};
use nalgebra::base::{Unit, Vector3};
use nalgebra::geometry::{Point2, Point3, Projective3};
use util::{color_to_luminance, cosine_hemisphere_pdf};

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

#[derive(PartialEq)]
// TODO: InfiniteAreaLights
pub enum Light {
    Point {
        flags: u8,
        to_world: Projective3<f64>,
        to_obj: Projective3<f64>,
        n_samples: u32,
        p: Point3<f64>,
        color: Vector3<f64>,
    },
    Spot {
        flags: u8,
        to_world: Projective3<f64>,
        to_obj: Projective3<f64>,
        n_samples: u32,
        p: Point3<f64>,
        color: Vector3<f64>,
        cos_width: f64,
        cos_falloff: f64,
    },
    Distant {
        flags: u8,
        to_world: Projective3<f64>,
        to_obj: Projective3<f64>,
        n_samples: u32,
        color: Vector3<f64>,
        dir: Vector3<f64>,
        world_center: Point3<f64>,
        world_radius: f64,
    },
    Diffuse {
        flags: u8,
        to_world: Projective3<f64>,
        to_obj: Projective3<f64>,
        n_samples: u32,
        color: Vector3<f64>,
        prim_index: usize,
        area: f64,
        two_sided: bool,
        is_mesh: bool,
    },
    Infinite {
        flags: u8,
        to_world: Projective3<f64>,
        to_obj: Projective3<f64>,
        n_samples: u32,
        text_id: usize,
        world_center: Point3<f64>,
        world_radius: f64,
        dist: Distribution2D,
    },
}

impl Light {
    pub fn get_flags(&self) -> u8 {
        match self {
            Light::Point { flags, .. } => *flags,
            Light::Spot { flags, .. } => *flags,
            Light::Distant { flags, .. } => *flags,
            Light::Diffuse { flags, .. } => *flags,
            Light::Infinite { flags, .. } => *flags,
        }
    }

    pub fn get_n_samples(&self) -> u32 {
        match self {
            Light::Point { n_samples, .. } => *n_samples,
            Light::Spot { n_samples, .. } => *n_samples,
            Light::Distant { n_samples, .. } => *n_samples,
            Light::Diffuse { n_samples, .. } => *n_samples,
            Light::Infinite { n_samples, .. } => *n_samples,
        }
    }

    pub fn get_inv_transform(&self) -> &Projective3<f64> {
        match self {
            Light::Point { to_obj, .. } => to_obj,
            Light::Spot { to_obj, .. } => to_obj,
            Light::Distant { to_obj, .. } => to_obj,
            Light::Diffuse { to_obj, .. } => to_obj,
            Light::Infinite { to_obj, .. } => to_obj,
        }
    }

    pub fn get_spot_cos(&self) -> (f64, f64) {
        match self {
            Light::Spot {
                cos_width,
                cos_falloff,
                ..
            } => (*cos_width, *cos_falloff),
            _ => (-1f64, -1f64), // doesn't matter
        }
    }

    pub fn is_delta_light(&self) -> bool {
        let flags = self.get_flags();
        return (flags & DELTA_POSITION) > 0 || (flags & DELTA_DIRECTION) > 0;
    }

    pub fn is_infinite(&self) -> bool {
        self.get_flags() & INFINITE > 0
    }

    /**
    Returns wi, pdf, color; illumination arriving at point from light
    */
    pub fn sample_li<'a, 'b, 'c>(
        &'a self,
        record: &'b HitRecord,
        u: &'c Point2<f64>,
    ) -> (Unit<Vector3<f64>>, f64, Vector3<f64>, Visibility<'b>) {
        match self {
            Light::Point { p, color, .. } => {
                let wi = Unit::new_normalize(p - record.p);
                let pdf = 1.;
                let new_record = HitRecord::make_basic(*p, record.t);
                let vis = Visibility::make_visibility(record, new_record);
                (
                    wi,
                    pdf,
                    color.scale(1. / nalgebra::distance_squared(p, &record.p)),
                    vis,
                )
            }
            Light::Spot { p, color, .. } => {
                let wi = Unit::new_normalize(p - record.p);
                let pdf = 1.;
                let new_record = HitRecord::make_basic(*p, record.t);
                let vis = Visibility::make_visibility(record, new_record);
                let color =
                    color.scale(falloff(self, &-wi) / nalgebra::distance_squared(p, &record.p));
                (wi, pdf, color, vis)
            }
            Light::Distant {
                color,
                dir,
                world_radius,
                ..
            } => {
                let wi = *dir;
                let pdf = 1.;
                let p_outside = record.p + dir * (2f64 * world_radius);
                let new_record = HitRecord::make_basic(p_outside, record.t);
                let vis = Visibility::make_visibility(record, new_record);
                (Unit::new_normalize(wi), pdf, *color, vis)
            }
            Light::Diffuse {
                prim_index,
                color,
                area,
                ..
            } => {
                let primitive = &get_objects().objs[*prim_index];
                let (mut new_record, mut pdf, wi) = primitive.sample(record, u);
                new_record.prim_index = *prim_index;
                if pdf == 0f64 || (record.p - new_record.p).magnitude_squared() == 0f64 {
                    pdf = 0f64;
                    // return dummy values
                    return (
                        Unit::new_normalize(util::black()),
                        pdf,
                        *color,
                        Visibility::make_visibility(
                            record,
                            HitRecord::make_basic(Point3::origin(), 1f64),
                        ),
                    );
                }
                let wi = Unit::new_normalize(new_record.p - record.p);
                // println!("Picked {}, dir from {} is ({}, {}, {})", new_record.p, record.p, wi.x, wi.y, wi.z);
                let l = self.l(&new_record, &-wi);
                let vis = Visibility::make_visibility(record, new_record);
                (wi, pdf, l, vis)
            }
            Light::Infinite {
                to_world,
                to_obj,
                n_samples,
                text_id,
                dist,
                world_radius,
                ..
            } => {
                let (uv, map_pdf) = dist.sample_continuous(u);
                if map_pdf == 0f64 {
                    // dummy
                    return (
                        Unit::new_normalize(util::black()),
                        0f64,
                        util::black(),
                        Visibility::make_visibility(
                            record,
                            HitRecord::make_basic(Point3::origin(), 1f64),
                        ),
                    );
                }
                // convert point to direction
                let theta = uv[1] * PI;
                let phi = uv[0] * 2f64 * PI;
                let cos_theta = theta.cos();
                let sin_theta = theta.sin();
                let cos_phi = phi.cos();
                let sin_phi = phi.sin();
                let v = Vector3::new(sin_theta * cos_phi, sin_theta * sin_phi, cos_theta);
                let wi = to_world.transform_vector(&v);
                let mut pdf = map_pdf / (2f64 * PI * PI * sin_theta); // jacobian from u, v coordinates to theta, phi
                if sin_theta == 0f64 {
                    pdf = 0f64;
                }
                let vis = Visibility::make_visibility(
                    record,
                    HitRecord::make_basic(record.p + wi.scale(2f64 * world_radius), record.t),
                );
                let color = Texture::value(*text_id, uv[0], uv[1], &Point3::origin());
                (Unit::new_normalize(wi), pdf, color, vis)
            }
        }
    }

    pub fn power(&self) -> Vector3<f64> {
        match self {
            Light::Point { color, .. } => 4. * PI * color,
            Light::Spot {
                color,
                cos_width,
                cos_falloff,
                ..
            } => color * 2f64 * PI * (1f64 - 0.5 * (cos_falloff + cos_width)),
            Light::Distant {
                color,
                world_radius,
                ..
            } => color * PI * *world_radius * *world_radius,
            Light::Diffuse {
                two_sided,
                color,
                area,
                ..
            } => (if *two_sided { 2f64 } else { 1f64 }) * color * *area * PI,
            Light::Infinite {
                text_id,
                world_radius,
                ..
            } => Texture::value(*text_id, 0.5, 0.5, &Point3::origin())
                .scale(PI * world_radius * world_radius),
        }
    }

    pub fn pdf_li(&self, record: &HitRecord, wi: &Vector3<f64>) -> f64 {
        let prims = &get_objects().objs;
        match self {
            Light::Point { .. } => 0.,
            Light::Spot { .. } => 0.,
            Light::Distant { .. } => 0.,
            Light::Diffuse { prim_index, .. } => prims[*prim_index].pdf(record, wi),
            Light::Infinite { to_world, dist, .. } => {
                let wi = to_world.transform_vector(wi);
                let theta = util::spherical_theta(&wi);
                let phi = util::spherical_phi(&wi);
                let sin_theta = theta.sin();
                if sin_theta == 0f64 {
                    return 0f64;
                }
                dist.pdf(&Point2::new(phi * INV_2PI, theta * INV_PI)) / (2f64 * PI * PI * sin_theta)
            }
        }
    }

    /**
    Amount of light leaving from light at direction;
    returns (pdf_pos, pdf_dir, ray, normal, color)
    */
    pub fn sample_le(
        &self,
        u1: &Point2<f64>,
        u2: &Point2<f64>,
        time: f64,
    ) -> (f64, f64, Ray, Vector3<f64>, Vector3<f64>) {
        match self {
            Light::Point { p, color, .. } => {
                let ray = Ray::new_time(*p, util::uniform_sample_sphere(u1), time);
                let n_light = ray.dir;
                let pdf_pos = 1f64;
                let pdf_dir = util::uniform_sphere_pdf();
                (pdf_pos, pdf_dir, ray, n_light, *color)
            }
            Light::Spot {
                color,
                to_world,
                p,
                cos_width,
                ..
            } => {
                let w = util::uniform_sample_cone(u1, *cos_width);
                let ray = Ray::new_time(*p, to_world.transform_vector(&w), time);
                let n_light = ray.dir;
                let pdf_pos = 1f64;
                let pdf_dir = util::uniform_cone_pdf(*cos_width);
                (pdf_pos, pdf_dir, ray, n_light, *color)
            }
            Light::Distant {
                color,
                dir,
                world_center,
                world_radius,
                ..
            } => {
                let onb = ONB::new_from_vec(dir);
                let cd = util::concentric_sample_disk(u1);
                let p_disk =
                    world_center + (onb.v().scale(cd.x) + onb.w().scale(cd.y)).scale(*world_radius);
                let ray = Ray::new_time(p_disk + dir * *world_radius, -dir, time);
                let n_light = ray.dir;
                let pdf_pos = 1f64 / (PI * *world_radius * *world_radius);
                let pdf_dir = 1f64;
                (pdf_pos, pdf_dir, ray, n_light, *color)
            }
            Light::Diffuse {
                color,
                prim_index,
                two_sided,
                ..
            } => {
                let objects: &Vec<Primitive> = &get_objects().objs;
                let (new_record, pdf_pos) = objects[*prim_index].sample_area(u1);
                let normal = new_record.n.into_inner();
                let mut w: Vector3<f64>;
                let pdf_dir: f64;
                if *two_sided {
                    let u = u2;
                    let mut u0 = u[0];
                    if u0 < 0.5 {
                        u0 = (u0 * 2f64).min(ONE_MINUS_EPSILON);
                        w = util::cosine_sample_hemisphere(&Point2::new(u0, u[1]));
                    } else {
                        u0 = ((u0 - 0.5) * 2f64).min(ONE_MINUS_EPSILON);
                        w = util::cosine_sample_hemisphere(&Point2::new(u0, u[1]));
                        w.z = w.z * -1f64;
                    }
                    pdf_dir = 0.5 * util::cosine_hemisphere_pdf(w.z.abs());
                } else {
                    w = util::cosine_sample_hemisphere(u2);
                    pdf_dir = util::cosine_hemisphere_pdf(w.z);
                }

                let (v1, v2) = util::make_coordinate_system(&normal);
                w = w.x * v1 + w.y * v2 + w.z * normal;
                let ray = new_record.spawn_ray(&w);
                (pdf_pos, pdf_dir, ray, normal, *color)
            }
            Light::Infinite {
                to_world,
                text_id,
                world_center,
                world_radius,
                dist,
                ..
            } => {
                let (uv, map_pdf) = dist.sample_continuous(u1);
                if map_pdf == 0f64 {
                    // dummy values
                    return (
                        0f64,
                        0f64,
                        Ray::new(Point3::origin(), util::black()),
                        util::black(),
                        util::black(),
                    );
                }
                let theta = uv[1] * PI;
                let phi = uv[0] * 2f64 * PI;
                let cos_theta = theta.cos();
                let sin_theta = theta.sin();
                let cos_phi = phi.cos();
                let sin_phi = phi.sin();
                // negative so that
                let d = -to_world.transform_vector(&Vector3::new(
                    sin_theta * cos_phi,
                    sin_theta * sin_phi,
                    cos_theta,
                ));

                let (v1, v2) = util::make_coordinate_system(&d);
                let cd = util::concentric_sample_disk(u2);
                let p_disk = world_center + (cd.x * v1 + cd.y * v2).scale(*world_radius);
                let ray = Ray::new(p_disk + -d.scale(*world_radius), d);
                // compute both pdfs
                let pdf_dir = if sin_theta == 0f64 {
                    0f64
                } else {
                    map_pdf / (2f64 * PI * PI * sin_theta) // jacobian
                };
                let pdf_pos = 1f64 / (PI * *world_radius * *world_radius);
                let color = Texture::value(*text_id, uv[0], uv[1], &Point3::origin());
                (pdf_pos, pdf_dir, ray, d, color)
            }
        }
    }

    /**
    Returns (pdf_pos, pdf_dir)
    */
    pub fn pdf_le(&self, ray: &Ray, normal: &Vector3<f64>) -> (f64, f64) {
        match self {
            Light::Point { .. } => (0f64, util::uniform_sphere_pdf()),
            Light::Spot { .. } => (0f64, util::uniform_sphere_pdf()),
            Light::Distant { world_radius, .. } => {
                (1f64 / (PI * *world_radius * *world_radius), 0f64)
            }
            Light::Diffuse {
                prim_index,
                two_sided,
                ..
            } => {
                let objects: &Vec<Primitive> = &get_objects().objs;
                let pdf_pos = objects[*prim_index].const_pdf();
                let pdf_dir = if *two_sided {
                    0.5 * cosine_hemisphere_pdf(normal.dot(&ray.dir).abs())
                } else {
                    cosine_hemisphere_pdf(normal.dot(&ray.dir))
                };
                (pdf_pos, pdf_dir)
            }
            Light::Infinite {
                flags,
                to_world,
                to_obj,
                n_samples,
                text_id,
                world_center,
                world_radius,
                dist,
            } => {
                let d = -to_obj.transform_vector(&ray.dir);
                let theta = util::spherical_theta(&d);
                let phi = util::spherical_phi(&d);
                let uv = Point2::new(phi * INV_2PI, theta * INV_PI);
                let map_pdf = dist.pdf(&uv);
                let pdf_dir = map_pdf / (2f64 * PI * PI * theta.sin());
                let pdf_pos = 1f64 / (PI * *world_radius * *world_radius);
                (pdf_pos, pdf_dir)
            }
        }
    }

    pub fn l(&self, record: &HitRecord, w: &Vector3<f64>) -> Vector3<f64> {
        match self {
            Light::Diffuse {
                flags,
                to_world,
                to_obj,
                n_samples,
                color,
                prim_index,
                area,
                two_sided,
                is_mesh,
            } => {
                if record.n.dot(w) > 0f64 || *two_sided {
                    *color
                } else {
                    util::black()
                }
            }
            _ => util::black(),
        }
    }

    // only nonzero for InfiniteAreaLights
    pub fn le(&self, ray: &Ray) -> Vector3<f64> {
        match self {
            Light::Infinite {
                to_obj, text_id, ..
            } => {
                let w = Unit::new_normalize(to_obj.transform_vector(&ray.dir));
                let uv = Point2::new(
                    util::spherical_phi(&w) * INV_2PI,
                    util::spherical_theta(&w) * INV_PI,
                );
                Texture::value(*text_id, uv[0], uv[1], &Point3::origin())
            }
            _ => util::black(),
        }
    }

    pub fn set_prim_index(&mut self, index: usize) {
        match self {
            Light::Diffuse { prim_index, .. } => *prim_index = index,
            _ => {}
        }
    }

    pub fn make_point_light(
        to_world: Projective3<f64>,
        color: Vector3<f64>,
        n_samples: u32,
    ) -> Self {
        let flags = DELTA_POSITION;
        let transformed_point = to_world.transform_point(&Point3::new(0., 0., 0.));
        Light::Point {
            flags,
            to_world,
            to_obj: to_world.inverse(),
            n_samples,
            p: transformed_point,
            color,
        }
    }

    pub fn make_spot_light(
        to_world: Projective3<f64>,
        color: Vector3<f64>,
        total_width: f64,
        falloff: f64,
        n_samples: u32,
    ) -> Self {
        let flags = DELTA_POSITION;
        let transformed_point = to_world.transform_point(&Point3::new(0., 0., 0.));
        let cos_width = (total_width * 180f64 * INV_PI).cos();
        let cos_falloff = (falloff * 180f64 * INV_PI).cos();
        Light::Spot {
            flags,
            to_world,
            to_obj: to_world.inverse(),
            p: transformed_point,
            color,
            cos_width,
            cos_falloff,
            n_samples,
        }
    }

    pub fn make_distant_light(
        to_world: Projective3<f64>,
        color: Vector3<f64>,
        dir: Vector3<f64>,
        n_samples: u32,
    ) -> Self {
        let flags = DELTA_DIRECTION;
        // TODO: Pick better world radius?
        Light::Distant {
            flags,
            to_world,
            to_obj: to_world.inverse(),
            color,
            dir,
            world_center: Point3::new(0., 0., 0.),
            world_radius: 1e10,
            n_samples,
        }
    }

    /**
    Remember to call set_prim_index
    */
    pub fn make_diffuse_light(
        prim_index: usize,
        to_world: Projective3<f64>,
        color: Vector3<f64>,
        n_samples: u32,
        two_sided: bool,
        is_mesh: bool,
    ) -> Self {
        let prim = &get_objects().objs[prim_index];
        let flags = AREA;
        Light::Diffuse {
            flags,
            to_world,
            to_obj: to_world.inverse(),
            n_samples,
            two_sided,
            color,
            prim_index,
            area: prim.area(),
            is_mesh,
        }
    }

    pub fn make_infinite_light(to_world: Projective3<f64>, n_samples: u32, text_id: usize) -> Self {
        let to_obj = to_world.inverse();
        let flags = INFINITE;
        let texture = &get_objects().textures[text_id];
        let dim = texture.get_size();
        let width = dim[0] as usize * 2;
        let height = dim[1] as usize * 2;
        let mut img = vec![0f64; width * height];
        let filter = 1f64 / (width.min(height) as f64);
        for v in 0usize..height {
            let vp = (v as f64 + 0.5) / (height as f64);
            let sin_theta = (PI * (v as f64 + 0.5) / (height as f64)).sin(); // don't fully understand this
            for u in 0usize..width {
                let up = (u as f64) / (width as f64);
                let color = texture.get_value(up, vp, &Point3::origin());
                img[u + v * width] = color_to_luminance(&color);
                img[u + v * width] = img[u + v * width] * sin_theta;
            }
        }
        let dist = Distribution2D::make_distribution_2d(&img, width, height);
        Light::Infinite {
            dist,
            n_samples,
            text_id,
            to_world,
            to_obj,
            flags,
            world_center: Point3::origin(),
            world_radius: 10000f64,
        }
    }
}
