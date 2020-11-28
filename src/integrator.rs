use nalgebra::base::Vector3;
use crate::geometry::Camera;
use crate::sampler::Samplers;
use crate::consts::*;
use crate::util;
use crate::geometry;

#[derive(Clone)]
pub enum Integrator {
    SamplerIntegrator {camera: Camera, sampler: Samplers },
    BasicSampler { camera: Camera }
}

impl Integrator {
    pub fn render(&self, grid: &mut Vec<Vec<(f64, f64, f64, u32)>>, px: u32, py: u32) {
        match self {
            Integrator::SamplerIntegrator { camera, sampler } => {}
            Integrator::BasicSampler { camera }=> {
                Integrator::basic_render(camera, grid, px, py);
            }
        }
    }

    pub fn basic_render(camera: &Camera, grid: &mut Vec<Vec<(f64, f64, f64, u32)>>, px: u32, py: u32) {
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
        let finished_pixel = grid[y as usize][x as usize];
    }

    pub fn make_sampler(camera: Camera, sampler: Samplers) -> Self {
        Integrator::SamplerIntegrator {camera, sampler }
    }
}