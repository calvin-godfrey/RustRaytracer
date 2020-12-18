#![allow(dead_code, unused_variables)] // TODO: Remove this
use nalgebra::base::Vector3;
use nalgebra::geometry::Point2;
use bumpalo::Bump;

use crate::{hittable::HitRecord, geometry::{Ray, Camera}};
use crate::sampler::Samplers;
use crate::consts::*;
use crate::util;
use crate::{geometry::get_objects};
use crate::geometry;
use crate::material::materials::Material;
use crate::light::Light;

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum LightStrategy {
    UniformAll,
    UniformOne
}

#[derive(Clone)]
pub enum IntType {
    Whitted { max_depth: u32 },
    Basic { max_depth: u32 },
    Direct { max_depth: u32, strategy: LightStrategy }
}

pub trait Integrator {
    fn render(&mut self, grid: &mut Vec<Vec<(f64, f64, f64, u32)>>, px: u32, py: u32);
    fn get_sampler(&mut self) -> &mut Samplers;
    fn init(&mut self);
}

pub trait SamplerIntegrator: Integrator {
    fn preprocess(&mut self);
    fn li(&mut self, ray: &Ray, depth: u32) -> Vector3<f64>;
}

pub fn get_integrator<'a>(int_type: IntType, camera: Camera, sampler: Samplers) -> Box<dyn Integrator> {
    match int_type {
        IntType::Whitted { max_depth } => {
            Box::new(WhittedIntegrator::make_whitted_integrator(camera, sampler, max_depth))
        }
        IntType::Basic { max_depth } => {
            Box::new(BasicIntegrator::make_basic_integrator(camera, sampler, max_depth))
        }
        IntType::Direct { max_depth, strategy } => {
            Box::new(DirectLightingIntegrator::make_direct_lighting(camera, sampler, max_depth, Vec::new(), strategy))
        }
    }
}

pub struct WhittedIntegrator {
    camera: Camera,
    sampler: Samplers,
    max_depth: u32
}

impl Integrator for WhittedIntegrator {
    fn render(&mut self, grid: &mut Vec<Vec<(f64, f64, f64, u32)>>, px: u32, py: u32) {
        let x = px % TILE_WIDTH;
        let y = py % TILE_HEIGHT;
        let seed = py * IMAGE_WIDTH + px;
        Samplers::start_pixel(&mut self.sampler, &Point2::new(px as i32, py as i32));
        loop {
            let (p_film, time, p_lens) = self.sampler.get_camera_sample(&Point2::new(px as i32, py as i32));
            // TODO: Allow lenses to change the weighting of ray based on lens
            let ray = self.camera.get_ray(p_film.x / (IMAGE_WIDTH as f64), p_film.y / (IMAGE_HEIGHT as f64));
            let color = self.li(&ray, self.max_depth);
            util::increment_color(grid, y as usize, x as usize, &color);
            if self.sampler.start_next_sample() == false {
                break;
            }
        }
    }

    fn get_sampler(&mut self) -> &mut Samplers { &mut self.sampler }
    fn init(&mut self) { self.preprocess(); }
}

impl SamplerIntegrator for WhittedIntegrator {
    fn preprocess(&mut self) {}

    fn li(&mut self, ray: &Ray, depth: u32) -> Vector3<f64> {
        let record = geometry::get_intersection(ray);
        let mut ans = util::black();
        let lights = &get_objects().lights;
        if record.is_none() {
            for light in lights {
                ans = ans + light.le(ray);
            }
            return ans
        }
        let mut record = record.unwrap();
        ans = record.le(&record.wo);
        let n = record.shading.n;
        let wo = record.wo;
        let arena = Bump::new();
        Material::compute_scattering_default(&mut record, &arena);
        for light in lights {
            let (wi, pdf, color, vis) = light.sample_li(&record, &self.sampler.get_2d());
            if color == util::black() || pdf == 0f64 {
                continue;
            }
            let f = record.bsdf.f_default(&wo, &wi);
            if f != util::black() && vis.unoccluded() {
                ans = ans + f.component_mul(&color) * wi.dot(&n).abs() / pdf;
            }
        }
        if depth - 1 > 0 {
            ans = ans + specular_reflect(self, ray, &record, depth);
            ans = ans + specular_transmit(self, ray, &record, depth);
        }
        ans
    }
}

impl WhittedIntegrator {
    pub fn make_whitted_integrator(camera: Camera, sampler: Samplers, max_depth: u32) -> Self {
        Self { camera, sampler, max_depth }
    }
}

pub struct BasicIntegrator {
    camera: Camera,
    sampler: Samplers,
    max_depth: u32
}

impl Integrator for BasicIntegrator {
    fn render(&mut self, grid: &mut Vec<Vec<(f64, f64, f64, u32)>>, px: u32, py: u32) {
        let x = px % TILE_WIDTH;
        let y = py % TILE_HEIGHT;
        // calculate the samples for the pixel
        for _ in 0..SAMPLES_PER_PIXEL {
            let u: f64 = (px as f64 + self.sampler.get_1d()) / ((IMAGE_WIDTH - 1) as f64);
            let v: f64 = (py as f64 + self.sampler.get_1d()) / ((IMAGE_HEIGHT - 1) as f64);
            let ray = self.camera.get_ray(u, v);
            let res: Vector3<f64> = geometry::cast_ray(&ray, self.max_depth);
            util::increment_color(grid, y as usize, x as usize, &res);
        }
    }

    fn get_sampler(&mut self) -> &mut Samplers { &mut self.sampler }
    fn init(&mut self) {}
}

impl BasicIntegrator {
    pub fn make_basic_integrator(camera: Camera, sampler: Samplers, max_depth: u32) -> Self {
        Self { camera, sampler, max_depth }
    }
}

struct DirectLightingIntegrator {
    camera: Camera,
    sampler: Samplers,
    max_depth: u32,
    n_light_samples: Vec<u32>,
    strategy: LightStrategy
}

impl Integrator for DirectLightingIntegrator {
    fn render(&mut self, grid: &mut Vec<Vec<(f64, f64, f64, u32)>>, px: u32, py: u32) {
        let x = px % TILE_WIDTH;
        let y = py % TILE_HEIGHT;
        let seed = py * IMAGE_WIDTH + px;
        Samplers::start_pixel(&mut self.sampler, &Point2::new(px as i32, py as i32));
        loop {
            let (p_film, time, p_lens) = self.sampler.get_camera_sample(&Point2::new(px as i32, py as i32));
            // TODO: Allow lenses to change the weighting of ray based on lens
            let ray = self.camera.get_ray(p_film.x / (IMAGE_WIDTH as f64), p_film.y / (IMAGE_HEIGHT as f64));
            let color = self.li(&ray, self.max_depth);
            util::increment_color(grid, y as usize, x as usize, &color);
            if self.sampler.start_next_sample() == false {
                break;
            }
        }
    }

    fn get_sampler(&mut self) -> &mut Samplers { &mut self.sampler }
    fn init(&mut self) { self.preprocess(); }
}

impl SamplerIntegrator for DirectLightingIntegrator {
    fn preprocess(&mut self) {
        let lights = &get_objects().lights;
        if self.strategy == LightStrategy::UniformAll {
            for light in lights {
                self.n_light_samples.push(self.sampler.round_count(light.get_n_samples()));
            }
        }
    }

    fn li(&mut self, ray: &Ray, depth: u32) -> Vector3<f64> {
        let record = geometry::get_intersection(ray);
        let mut ans = util::black();
        let lights = &get_objects().lights;
        if record.is_none() {
            for light in lights {
                ans = ans + light.le(ray);
            }
            return ans
        }
        let mut record = record.unwrap();
        let wo = &record.wo;
        ans = record.le(wo);
        let arena = Bump::new();
        Material::compute_scattering_default(&mut record, &arena);
        let lights = &get_objects().lights;
        if lights.len() > 0 {
            if self.strategy == LightStrategy::UniformAll {
                ans = ans + uniform_sample_all_lights(&record, &mut self.sampler, &self.n_light_samples, false);
            } else {
                ans = ans + uniform_sample_one_light(&record, &mut self.sampler, false);
            }
        }
        if depth - 1 > 0 {
            ans = ans + specular_reflect(self, ray, &record, depth);
            ans = ans + specular_transmit(self, ray, &record, depth);
        }
        ans
    }
}

impl DirectLightingIntegrator {
    pub fn make_direct_lighting(camera: Camera, sampler: Samplers, max_depth: u32, n_light_samples: Vec<u32>, strategy: LightStrategy) -> Self {
        Self { camera, sampler, max_depth, n_light_samples, strategy }
    }
}

fn specular_reflect(integrator: &mut dyn SamplerIntegrator, ray: &Ray, hit_record: &HitRecord, depth: u32) -> Vector3<f64> {
    let sampler = integrator.get_sampler();
    let wo = hit_record.wo;
    let bxdf_type = BSDF_REFLECTION | BSDF_SPECULAR;
    let (color, wi, pdf, new_type) = hit_record.bsdf.sample_f(&wo, &sampler.get_2d(), bxdf_type);

    let ns = hit_record.shading.n;

    if pdf > 0f64 && color != util::black() && ns.dot(&wi).abs() != 0f64 {
        let new_ray = hit_record.spawn_ray(&wi);
        return color.component_mul(&integrator.li(&new_ray, depth - 1)).scale(ns.dot(&wi).abs() / pdf);
    }
    util::black()
}

fn specular_transmit(integrator: &mut dyn SamplerIntegrator, ray: &Ray, hit_record: &HitRecord, depth: u32) -> Vector3<f64> {
    let sampler = integrator.get_sampler();
    let wo = hit_record.wo;
    let bxdf_type = BSDF_TRANSMISSION | BSDF_SPECULAR;
    let (color, wi, pdf, new_type) = hit_record.bsdf.sample_f(&wo, &sampler.get_2d(), bxdf_type);
    let ns = hit_record.shading.n;
    if pdf > 0f64 && color != util::black() && ns.dot(&wi).abs() != 0f64 {
        let new_ray = hit_record.spawn_ray(&wi);
        return color.component_mul(&integrator.li(&new_ray, depth - 1)).scale(ns.dot(&wi).abs() / pdf);
    }
    util::black()
}

fn uniform_sample_all_lights(record: &HitRecord, sampler: &mut Samplers, n_samples: &[u32], handle_media: bool) -> Vector3<f64> {
    let handle_media = false; // TODO: media
    let mut ans = util::black();
    for j in 0usize..get_objects().lights.len() {
        let light = &get_objects().lights[j];
        let samples = n_samples[j];
        let mut ld = util::black();
        // for _ in 0..samples {
            let u_light = sampler.get_2d(); // TODO request array ahead of time
            let u_scatter = sampler.get_2d();
            ld = ld + estimate_direct_default(record, &u_scatter, light, &u_light, sampler, handle_media);
        // }
        // ld = ld.scale(1f64 / (samples as f64));
        ans = ans + ld;
    }
    ans
}

fn uniform_sample_one_light(record: &HitRecord, sampler: &mut Samplers, handle_media: bool) -> Vector3<f64> {
    let handle_media = false;
    // choose single light to sample
    let lights = &get_objects().lights;
    let n_lights = lights.len();
    if n_lights == 0 {
        return util::black(); // no light
    }
    let light_num = (n_lights - 1).min((sampler.get_1d() * (n_lights as f64)) as usize);
    let light = &lights[light_num];
    let u_light = sampler.get_2d();
    let u_scattering = sampler.get_2d();
    // weird stat magic, somehow E[f(x) + g(x)] = E[2f(x)] on average or something?
    // that's why we multiply by n_lights
    estimate_direct_default(record, &u_scattering, light, &u_light, sampler, handle_media).scale(n_lights as f64)
}

fn estimate_direct(record: &HitRecord, u_scatter: &Point2<f64>, light: &Light, u_light: &Point2<f64>, sampler: &mut Samplers, handle_media: bool, specular: bool) -> Vector3<f64> {
    let bsdf_flags = if specular {
        BSDF_ALL
    } else {
        BSDF_ALL - BSDF_SPECULAR
    };
    let mut ld = util::black();
    let (wi, mut light_pdf, mut color, vis) = light.sample_li(record, u_light);
    let scattering_pdf: f64;
    // if we get a successful sample from the light distribution, then we evaluate the BSDF
    // to determine the actual contribution of light
    if light_pdf > 0f64 && color != util::black() {
        let f: Vector3<f64> = record.bsdf.f(&record.wo, &wi, bsdf_flags) * wi.dot(&record.shading.n).abs();
        scattering_pdf = record.bsdf.pdf(&record.wo, &wi, bsdf_flags);
        if f != util::black() {
            // TODO: use handle_media
            if !vis.unoccluded() {
                color = util::black();
            }
            if color != util::black() {
                if light.is_delta_light() {
                    ld = ld + f.component_mul(&color).scale(1f64/light_pdf);
                } else {
                    let weight = power_heuristic(1, light_pdf, 1, scattering_pdf);
                    ld = ld + f.component_mul(&color).scale(weight / light_pdf);
                }
            }
        }
    }

    // Then, if there isn't a delta light, we sample_f on the bsdf to get an outgoing direction and
    // add the contribution of the light it hits (if any)
    if !light.is_delta_light() {
        let (f, wi, scattering_pdf, sampled_type) = record.bsdf.sample_f(&record.wo, u_scatter, bsdf_flags);
        let f: Vector3<f64> = f * wi.dot(&record.shading.n).abs();
        let sampled_specular = (sampled_type & BSDF_SPECULAR) != 0;
        if f != util::black() && scattering_pdf > 0f64 {
            let mut weight = 1f64;
            if !sampled_specular {
                light_pdf = light.pdf_li(record, &wi);
                if light_pdf == 0f64 {
                    return ld;
                }
                weight = power_heuristic(1, scattering_pdf, 1, light_pdf);
            }

            let new_ray = record.spawn_ray(&wi);
            let new_record = geometry::get_intersection(&new_ray);
            let mut color = util::black();
            if new_record.is_some() {
                let new_record = new_record.unwrap();
                let index = &get_objects().objs[new_record.prim_index].get_light_index();
                if *index != std::usize::MAX {
                    if get_objects().lights[*index].eq(light) {
                        color = new_record.le(&-wi);
                    }
                }
            } else {
                color = light.le(&new_ray);
            }
            if color != util::black() {
                ld = ld + f.component_mul(&color).scale(weight / scattering_pdf);
            }
        }
    }
    ld
}

fn estimate_direct_default(record: &HitRecord, u_scatter: &Point2<f64>, light: &Light, u_light: &Point2<f64>, sampler: &mut Samplers, handle_media: bool) -> Vector3<f64> {
    estimate_direct(record, u_scatter, light, u_light, sampler, handle_media, false)
}

fn power_heuristic(nf: i32, f_pdf: f64, ng: i32, g_pdf: f64) -> f64 {
    let f = nf as f64 * f_pdf;
    let g = ng as f64 * g_pdf;
    (f * f) / (f * f + g * g)
}
