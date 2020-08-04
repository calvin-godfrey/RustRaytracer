use nalgebra::{base::Vector3, geometry::Point3};

use crate::geometry::ONB;
use crate::consts::*;
use crate::util;
use crate::primitive::Primitive;

#[derive(Clone)]
pub enum PDF {
    CosPdf { onb: ONB },
    HittablePdf { index: usize, origin: Point3<f64> },
    MixturePdf { a: Box<PDF>, b: Box<PDF> }
}

impl PDF {
    #[allow(unused_variables)]
    pub fn value(pdf: &PDF, objs: &Vec<Primitive>, dir: &Vector3<f64>) -> f64 {
        match pdf {
            PDF::CosPdf { onb } => {
                let cos = dir.dot(&onb.w()) / dir.magnitude();
                (cos / PI).max(0.)
            }
            PDF::HittablePdf { index, origin } => {
                Primitive::get_pdf(&objs[*index], origin, dir)
            }
            PDF::MixturePdf { a, b } => {
                0.5 * PDF::value(a.as_ref(), objs, dir) + 0.5 * PDF::value(b.as_ref(), objs, dir)
            }
        }
    }

    pub fn generate(pdf: &PDF, objs: &Vec<Primitive>) -> Vector3<f64> {
        match pdf {
            PDF::CosPdf { onb } => {
                onb.get_local_vec(&util::rand_cosine_dir())
            }
            PDF::HittablePdf { index, origin } => {
                Primitive::get_rand_dir(&objs[*index], origin)
            }
            PDF::MixturePdf { a, b } => {
                if util::rand() < 0.5 {
                    PDF::generate(a.as_ref(), objs)
                } else {
                    PDF::generate(b.as_ref(), objs)
                }
            }
        }
    }

    pub fn new_cos(dir: &Vector3<f64>) -> Self {
        let onb = ONB::new_from_vec(dir);
        PDF::CosPdf { onb }
    }

    pub fn new_hittable(index: usize, origin: Point3<f64>) -> Self {
        PDF::HittablePdf { index, origin }
    }

    pub fn new_mixture(a: PDF, b: PDF) -> Self {
        PDF::MixturePdf { a: Box::new(a), b: Box::new(b) }
    }
}