use nalgebra::base::Vector3;
use nalgebra::geometry::Point2;
use rand::prelude::*;
use crate::{hittable::HitRecord, geometry::{Ray, Camera}};
use crate::sampler::Samplers;
use crate::consts::*;
use crate::util;
use crate::geometry;

#[derive(Clone)]
pub enum Integrator {
    SamplerIntegrator {camera: Camera, sampler: Samplers },
    BasicSampler { camera: Camera }
}

enum IntegratorType {
    Sampler,
    Basic,
}

impl Integrator {

    fn pre_process(integrator: &Integrator) {}

    pub fn render(&mut self, grid: &mut Vec<Vec<(f64, f64, f64, u32)>>, px: u32, py: u32) {
        Integrator::pre_process(self);
        match self {
            Integrator::SamplerIntegrator { camera, sampler } => {
                let x = px % TILE_WIDTH;
                let y = py % TILE_HEIGHT;
                let seed = py * IMAGE_WIDTH + px;
                let mut rng = rand_pcg::Pcg64::seed_from_u64(seed as u64);
                Samplers::start_pixel(sampler, &Point2::new(px as i32, py as i32));
                loop {
                    let (p_film, time, p_lens) = Samplers::get_camera_sample(sampler, &Point2::new(px as i32, py as i32));
                    // TODO: Allow lenses to change the weighting of ray based on lens
                    let ray = camera.get_ray(p_film.x / (IMAGE_WIDTH as f64), p_film.y / (IMAGE_HEIGHT as f64));
                    let color = Integrator::li(IntegratorType::Sampler, sampler, &mut rng, &ray, MAX_DEPTH);
                    util::increment_color(grid, y as usize, x as usize, &color);
                    if Samplers::start_next_sample(sampler) == false {
                        break;
                    }
                }

            }
            Integrator::BasicSampler { camera }=> {
                Integrator::basic_render(camera, grid, px, py);
            }
        }
    }

    fn li(int_type: IntegratorType, sampler: &mut Samplers, rng: &mut rand_pcg::Lcg128Xsl64, ray: &Ray, depth: u32) -> Vector3<f64> {
        geometry::cast_ray(&ray, depth)
    }

    fn basic_render(camera: &Camera, grid: &mut Vec<Vec<(f64, f64, f64, u32)>>, px: u32, py: u32) {
        let x = px % TILE_WIDTH;
        let y = py % TILE_HEIGHT;
        // calculate the samples for the pixel
        for _ in 0..SAMPLES_PER_PIXEL {
            let u: f64 = (px as f64 + util::rand()) / ((IMAGE_WIDTH - 1) as f64);
            let v: f64 = (py as f64 + util::rand()) / ((IMAGE_HEIGHT - 1) as f64);
            let ray = camera.get_ray(u, v);
            let res: Vector3<f64> = geometry::cast_ray(&ray, MAX_DEPTH);
            util::increment_color(grid, y as usize, x as usize, &res);
        }
    }

    pub fn make_sampler_integrator(camera: Camera, sampler: Samplers) -> Self {
        Integrator::SamplerIntegrator { camera, sampler }
    }

    pub fn make_random_integrator(camera: Camera) -> Self {
        Integrator::BasicSampler {camera}
    }
}

fn specular_reflect(int_type: IntegratorType, ray: &Ray, hit_record: &HitRecord, sampler: &mut Samplers, depth: u32, rng: &mut rand_pcg::Lcg128Xsl64) -> Vector3<f64> {
    let wo = hit_record.wo;
    let bxdf_type = BSDF_REFLECTION | BSDF_SPECULAR;
    let (color, wi, pdf, new_type) = hit_record.bsdf.sample_f(&wo, &sampler.get_2d(), bxdf_type);

    let ns = hit_record.shading.n;

    if pdf > 0f64 && color != util::black() && ns.dot(&wi).abs() != 0f64 {
        let new_ray = hit_record.spawn_ray(&wi);
        return color.component_mul(&Integrator::li(int_type, sampler, rng, &new_ray, depth - 1)).scale(ns.dot(&wi) / pdf);
    }
    util::black()
}

fn specular_transmssion(int_type: IntegratorType, ray: &Ray, hit_record: &HitRecord, sampler: &mut Samplers, depth: u32, rng: &mut rand_pcg::Lcg128Xsl64) -> Vector3<f64> {
    let wo = hit_record.wo;
    let bxdf_type = BSDF_TRANSMISSION | BSDF_SPECULAR;
    let (color, wi, pdf, new_type) = hit_record.bsdf.sample_f(&wo, &sampler.get_2d(), bxdf_type);
    let ns = hit_record.shading.n;
    if pdf > 0f64 && color != util::black() && ns.dot(&wi).abs() != 0f64 {
        let new_ray = hit_record.spawn_ray(&wi);
        return color.component_mul(&Integrator::li(int_type, sampler, rng, &new_ray, depth - 1)).scale(ns.dot(&wi).abs() / pdf);
    }
    util::black()
}