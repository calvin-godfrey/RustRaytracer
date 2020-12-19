#![allow(dead_code)]
use nalgebra::{geometry::Point2, base::{Unit, Vector3}};
use crate::bxdf;
use crate::hittable::HitRecord;
use crate::util;
use crate::consts::*;

const EMPTY_ETA: f64 = -100.;

pub struct Bsdf {
    pub eta: f64,
    ns: Unit<Vector3<f64>>, // shading normal
    ng: Unit<Vector3<f64>>, // geometric normal
    ss: Unit<Vector3<f64>>, // shading tangent
    ts: Unit<Vector3<f64>>, // shading bitangent
    bxdfs: Vec<Box<bxdf::Bxdf>>,
}

impl Bsdf {
    /** Constructor takes a hitrecord and index of refraction; if index of refraction
    is not applicable, like for an opaque surface, pass a value of 1.
    */
    pub fn new(record: &HitRecord, eta: f64) -> Self {
        Bsdf { eta, ns: record.shading.n, ng: record.n, ss: record.shading.dpdu, ts: Unit::new_normalize(record.shading.n.cross(&record.shading.dpdu)), bxdfs: vec![]}
    }

    pub fn empty() -> Self {
        let n = Unit::new_normalize(util::white());
        Self { eta: EMPTY_ETA, ns: n, ng: n, ss: n, ts: n, bxdfs: vec![]}
    }

    pub fn is_empty(&self) -> bool {
        self.eta == EMPTY_ETA
    }

    pub fn add(&mut self, bxdf: bxdf::Bxdf) {
        self.bxdfs.push(Box::new(bxdf));
    }

    pub fn num_components(&self, bxdf_type: u8) -> u32{
        let mut cnt: u32 = 0;
        for i in 0..self.bxdfs.len() {
            if bxdf::Bxdf::matches_flag(self.bxdfs[i].as_ref(), bxdf_type) {
                cnt += 1;
            }
        }
        cnt
    }

    pub fn world_to_local(&self, v: &Vector3<f64>) -> Vector3<f64> {
        Vector3::new(v.dot(&self.ss), v.dot(&self.ts), v.dot(&self.ns))
    }

    pub fn local_to_world(&self, v: &Vector3<f64>) -> Vector3<f64> {
        Vector3::new(self.ss.x * v.x + self.ts.x * v.y + self.ns.x * v.z,
                     self.ss.y * v.x + self.ts.y * v.y + self.ns.y * v.z,
                        self.ss.z * v.x + self.ts.z * v.y + self.ns.z * v.z)
    }

    pub fn f_default(&self, wow: &Vector3<f64>, wiw: &Vector3<f64>) -> Vector3<f64> {
        self.f(wow, wiw, BSDF_ALL)
    }

    pub fn f(&self, wow: &Vector3<f64>, wiw: &Vector3<f64>, flags: u8) -> Vector3<f64> {
        let wi = self.world_to_local(wiw);
        let wo = self.world_to_local(wow);
        let reflect = wiw.dot(&self.ng) * wow.dot(&self.ng) > 0.;
        let mut f = util::black();
        for i in 0..self.bxdfs.len() {
            let b = self.bxdfs[i].as_ref();
            if bxdf::matches_flags(b.get_type(), flags) &&
                 ((reflect && (b.get_type() & BSDF_REFLECTION) > 0)) ||
                 (!reflect && (b.get_type() & BSDF_TRANSMISSION) > 0) {
                    f += bxdf::Bxdf::f(b, &wo, &wi);
            }
        }
        f
    }

    /** Returns color, new direction, pdf, type
    */
    pub fn sample_f(&self, wow: &Vector3<f64>, sample: &Point2<f64>, bxdf_type: u8) -> (Vector3<f64>, Vector3<f64>, f64, u8) {
        let matching = self.num_components(bxdf_type);
        if matching == 0 {
            return (util::black(), util::black(), 0f64, 0u8);
        }
        let comp = ((sample[0] * matching as f64).floor() as u32).min(matching - 1);
        let mut count = comp;
        let mut bxdf: &bxdf::Bxdf = self.bxdfs[0].as_ref();
        let mut used_index: usize = std::usize::MAX;
        for index in 0..self.bxdfs.len() {
            bxdf = self.bxdfs[index].as_ref();
            if bxdf::matches_flags(bxdf.get_type(), bxdf_type) {
                if count == 0 {
                    used_index = index;
                    break;
                }
                count -= 1;
            }
        }
        assert!(used_index != std::usize::MAX);
        let wo = Unit::new_normalize(self.world_to_local(wow)).into_inner();
        if wo.z == 0. {
            (util::black(), util::black(), 0f64, 0u8);
        }
        let (mut color, wi, mut pdf) = bxdf::Bxdf::sample_f(bxdf, &wo, sample, bxdf_type);
        if pdf == 0. {
            (util::black(), util::black(), 0f64, 0u8);
        }
        let wiw = self.local_to_world(&wi);
        
        if (bxdf.get_type() & BSDF_SPECULAR) == 0 && matching > 1 {
            for index in 0..self.bxdfs.len() {
                let b = self.bxdfs[index].as_ref();
                if index != used_index && bxdf::matches_flags(bxdf::Bxdf::get_type(b), bxdf_type) {
                    pdf += bxdf::Bxdf::pdf(b, &wo, &wi);
                }
            }
        }
        if matching > 1 {
            pdf = pdf / matching as f64;
        }
        
        if (bxdf::Bxdf::get_type(bxdf) & BSDF_SPECULAR) == 0 {
            let reflect = wiw.dot(&self.ng) * wow.dot(&self.ng) > 0.;
            color = Vector3::new(0., 0., 0.);
            for index in 0..self.bxdfs.len() {
                let b = self.bxdfs[index].as_ref();
                if bxdf::matches_flags(bxdf::Bxdf::get_type(b), bxdf_type) &&
                ((reflect && (bxdf::Bxdf::get_type(b) & BSDF_REFLECTION) > 0) || 
                ((!reflect && (bxdf::Bxdf::get_type(b) & BSDF_TRANSMISSION) > 0))) {
                    color += bxdf::Bxdf::f(b, &wo, &wi);
                }
            }
        }
        
        (color, wiw, pdf, 0u8)
    }

    pub fn pdf(&self, wow: &Vector3<f64>, wiw: &Vector3<f64>, flags: u8) -> f64 {
        let n_components = self.num_components(BSDF_ALL);
        if n_components == 0 {
            return 0f64;
        }
        let wo = self.world_to_local(&wow);
        let wi = self.world_to_local(&wiw);
        if wo.z == 0f64 {
            return 0f64;
        }
        let mut pdf = 0f64;
        let mut matching: i32 = 0;
        for i in 0usize..(n_components as usize) {
            if self.bxdfs[i].matches_flag(flags) {
                matching += 1;
                pdf += self.bxdfs[i].pdf(&wo, &wi);
            }
        }
        if matching > 0 {
            pdf
        } else {
            0f64
        }
    }
    // TODO: Add rho methods
}