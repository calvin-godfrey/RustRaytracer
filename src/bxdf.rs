use crate::consts::*;
use crate::microfacet::MicrofacetDistribution;
use crate::util;
use nalgebra::{
    base::{Unit, Vector3},
    geometry::Point2,
};

// this is calculated in the coordinate system where
// theta is the angle down from z+, and phi is
// the angle formed with x when the vector is projected down
pub fn cos_theta(v: &Vector3<f64>) -> f64 {
    v.z
}
pub fn cos_sq_theta(v: &Vector3<f64>) -> f64 {
    v.z * v.z
}
pub fn abs_cos_theta(v: &Vector3<f64>) -> f64 {
    v.z.abs()
}
pub fn sin_theta(v: &Vector3<f64>) -> f64 {
    sin_sq_theta(v).sqrt()
}
pub fn sin_sq_theta(v: &Vector3<f64>) -> f64 {
    0f64.max(1f64 - cos_sq_theta(v))
}
pub fn tan_theta(v: &Vector3<f64>) -> f64 {
    sin_theta(v) / cos_theta(v)
}
pub fn tan_sq_theta(v: &Vector3<f64>) -> f64 {
    sin_sq_theta(v) / cos_sq_theta(v)
}

pub fn cos_phi(v: &Vector3<f64>) -> f64 {
    let sin_t = sin_theta(v);
    return if sin_t == 0. {
        1.
    } else {
        util::clamp(v.x / sin_t, -1., 1.)
    };
}
pub fn cos_sq_phi(v: &Vector3<f64>) -> f64 {
    cos_phi(v) * cos_phi(v)
}

pub fn sin_phi(v: &Vector3<f64>) -> f64 {
    let sin_t = sin_theta(v);
    return if sin_t == 0. {
        1.
    } else {
        util::clamp(v.y / sin_t, -1., 1.)
    };
}
pub fn sin_sq_phi(v: &Vector3<f64>) -> f64 {
    sin_phi(v) * sin_phi(v)
}
pub fn cos_d_phi(va: &Vector3<f64>, vb: &Vector3<f64>) -> f64 {
    return util::clamp(
        (va.x * vb.x + va.y * vb.y)
            / ((va.x * va.x + va.y * va.y) * (vb.x * vb.x + vb.y * vb.y)).sqrt(),
        -1.,
        1.,
    );
}

pub fn matches_flags(flag: u8, other_flag: u8) -> bool {
    (flag & other_flag) == flag
}

fn schlick_fresnel(r_s: &Vector3<f64>, cos_theta: f64) -> Vector3<f64> {
    r_s.component_mul(&(util::white() - r_s).scale((1. - cos_theta).powf(5.)))
}

fn schlick_weight(u: f64) -> f64 {
    let m = util::clamp(1f64 - u, 0f64, 1f64);
    let m2 = m * m;
    m2 * m2 * m
}

fn fr_schlick(r0: &Vector3<f64>, cos_theta: f64) -> Vector3<f64> {
    let weight = schlick_weight(cos_theta);
    // linearly interpolate
    // util::lerp_v(weight, r0, &util::white())
    r0 + weight * (util::white() - r0)
}

fn gtr1(cos_theta: f64, a: f64) -> f64 {
    if cos_theta > 1f64 {
        INV_PI
    } else {
        let a2 = a * a;
        (a2 - 1f64) / (PI * a2.ln() * (1f64 + (a2 - 1f64) * cos_theta * cos_theta))
    }
}

fn smithg_ggx(cos_theta: f64, alpha: f64) -> f64 {
    let a2 = alpha * alpha;
    let cos_theta_2 = cos_theta * cos_theta;
    // 1f64 / (cos_theta + (a2 + cos_theta_2 - a2 * cos_theta_2).sqrt())
    2f64 / (1f64 + (a2 + (1f64 - a2) * cos_theta_2).sqrt())
}

pub fn eta_to_r0(eta: f64) -> f64 {
    (eta - 1f64) * (eta - 1f64) / ((eta + 1f64) * (eta + 1f64))
}

/** Calculates the fraction of light that is reflected or transmitted
based on the cosine of the angle between the incoming light and surface normal
and the two indices of refraction. Assumes that eta_i is the index of the incident material,
and that the eta_i and eta_t are both real valued.
Note that the fraction of light transmitted is 1 - fr_dielectric().
*/
pub fn fr_dielectric(cos_theta_i: f64, eta_i: f64, eta_t: f64) -> f64 {
    let mut cos_theta_i = util::clamp(cos_theta_i, -1., 1.);
    let mut index_i = eta_i;
    let mut index_t = eta_t;
    if cos_theta_i < 0. {
        // leaving the surface
        index_i = eta_t;
        index_t = eta_i;
        cos_theta_i = cos_theta_i.abs();
    }
    // calculate terms with snell's law
    let sin_theta_i = 0f64.max(1f64 - cos_theta_i * cos_theta_i).sqrt();
    let sin_theta_t = index_i / index_t * sin_theta_i;
    let cos_theta_t = 0f64.max(1f64 - sin_theta_t * sin_theta_t).sqrt();
    if sin_theta_t >= 1. {
        // internal reflection
        return 1.;
    }
    let r_parl: f64 = ((index_t * cos_theta_i) - (index_i * cos_theta_t))
        / ((index_t * cos_theta_i) + (index_i * cos_theta_t));
    let r_perp: f64 = ((index_i * cos_theta_i) - (index_t * cos_theta_t))
        / ((index_i * cos_theta_i) + (index_t * cos_theta_t));
    (r_parl * r_parl + r_perp * r_perp) / 2.
}

// TODO: Implement Fresnel reflection for conductors (metals). It (might?) require
// spectral path tracing because it is wavelength dependent.

pub fn fr_conductor(cos_theta_i: f64, eta: &Vector3<f64>, eta_k: &Vector3<f64>) -> Vector3<f64> {
    let cos_theta_i = util::clamp(cos_theta_i, -1., 1.);
    let cos_theta_i2 = cos_theta_i * cos_theta_i;
    let sin_theta_i2 = 1f64 - cos_theta_i2;
    let eta2 = eta.component_mul(&eta);
    let etak2 = eta_k.component_mul(&eta_k);

    let t0 = (eta2 - etak2) - Vector3::new(sin_theta_i2, sin_theta_i2, sin_theta_i2);
    let a2_plus_b2: Vector3<f64> = t0.component_mul(&t0) + eta2.component_mul(&etak2).scale(4f64);
    let a2_plus_b2 = Vector3::new(
        a2_plus_b2.x.sqrt(),
        a2_plus_b2.y.sqrt(),
        a2_plus_b2.z.sqrt(),
    );
    let t1 = a2_plus_b2 + Vector3::new(cos_theta_i2, cos_theta_i2, cos_theta_i2);
    let a = (a2_plus_b2 + t0).scale(0.5);
    let a = Vector3::new(a.x.sqrt(), a.y.sqrt(), a.z.sqrt());
    let t2 = a.scale(2f64 * cos_theta_i);
    let rs = (t1 - t2).component_div(&(t1 + t2));

    let t3 = a2_plus_b2.scale(cos_theta_i2)
        + Vector3::new(
            sin_theta_i2 * sin_theta_i2,
            sin_theta_i2 * sin_theta_i2,
            sin_theta_i2 * sin_theta_i2,
        );
    let t4 = t2.scale(sin_theta_i2);
    let rp = rs.component_mul(&((t3 - t4).component_div(&(t3 + t4))));
    (rp + rs).scale(0.5)
}

#[derive(Copy, Clone)]
pub enum Fresnel {
    FresnelDielectric {
        eta_i: f64,
        eta_t: f64,
    },
    FresnelConductor {
        eta: Vector3<f64>,
        k: Vector3<f64>,
    },
    DisneyFresnel {
        r0: Vector3<f64>,
        metallic: f64,
        eta: f64,
    },
    FresnelNoOp, // not physically possible, but convenient to have
}

impl Fresnel {
    pub fn evaluate(fresnel: &Fresnel, cos_theta_i: f64) -> Vector3<f64> {
        match fresnel {
            Fresnel::FresnelDielectric { eta_i, eta_t } => {
                let value = fr_dielectric(cos_theta_i.abs(), *eta_t, *eta_i);
                Vector3::new(value, value, value)
            }
            Fresnel::FresnelNoOp => util::white(),
            Fresnel::FresnelConductor { eta, k } => fr_conductor(cos_theta_i.abs(), &eta, k),
            Fresnel::DisneyFresnel { r0, metallic, eta } => {
                let min = fr_dielectric(cos_theta_i, 1f64, *eta);
                let max = fr_schlick(r0, cos_theta_i);
                let ans = Vector3::new(
                    util::lerp(*metallic, min, max.x),
                    util::lerp(*metallic, min, max.y),
                    util::lerp(*metallic, min, max.z),
                );
                ans
            }
        }
    }
}

pub enum Bxdf {
    ScaledBxdf {
        inner: Box<Bxdf>,
        scale: Vector3<f64>,
    }, // scale is color
    // fresnel is so that we can choose between dielectric and conductor
    SpecularReflection {
        color: Vector3<f64>,
        fresnel: Fresnel,
        bxdf_type: u8,
    },
    SpecularTransmission {
        color: Vector3<f64>,
        eta_a: f64,
        eta_b: f64,
        fresnel: Fresnel,
        mode: u8,
        bxdf_type: u8,
    },
    // TODO: Add fresnel specular
    LambertianReflection {
        color: Vector3<f64>,
        bxdf_type: u8,
    },
    LambertianTransmission {
        color: Vector3<f64>,
        bxdf_type: u8,
    },
    OrenNayer {
        color: Vector3<f64>,
        a: f64,
        b: f64,
        bxdf_type: u8,
    },
    MicrofacetReflection {
        color: Vector3<f64>,
        fresnel: Fresnel,
        mfd: MicrofacetDistribution,
        bxdf_type: u8,
    },
    MicrofacetTransmission {
        color: Vector3<f64>,
        fresnel: Fresnel,
        mfd: MicrofacetDistribution,
        eta_a: f64,
        eta_b: f64,
        mode: u8,
        bxdf_type: u8,
    },
    FresnelBlend {
        r_s: Vector3<f64>,
        r_d: Vector3<f64>,
        mfd: MicrofacetDistribution,
        bxdf_type: u8,
    },
    FresnelSpecular {
        r: Vector3<f64>,
        t: Vector3<f64>,
        eta_a: f64,
        eta_b: f64,
        mode: u8,
        bxdf_type: u8,
    },
    // and here comes all the disney stuff for the disney brdf
    DisneyDiffuse {
        color: Vector3<f64>,
        bxdf_type: u8,
    },
    DisneyFakeSS {
        color: Vector3<f64>,
        roughness: f64,
        bxdf_type: u8,
    },
    DisneyRetro {
        color: Vector3<f64>,
        roughness: f64,
        bxdf_type: u8,
    },
    DisneySheen {
        color: Vector3<f64>,
        bxdf_type: u8,
    },
    DisneyClearCoat {
        weight: f64,
        gloss: f64,
        bxdf_type: u8,
    },
}

#[allow(unused_variables)]
impl Bxdf {
    pub fn matches_flag(&self, flag: u8) -> bool {
        match &self {
            Bxdf::ScaledBxdf { inner, .. } => Bxdf::matches_flag(inner.as_ref(), flag),
            Bxdf::SpecularReflection { bxdf_type, .. } => matches_flags(*bxdf_type, flag),
            Bxdf::SpecularTransmission { bxdf_type, .. } => matches_flags(*bxdf_type, flag),
            Bxdf::LambertianReflection { bxdf_type, .. } => matches_flags(*bxdf_type, flag),
            Bxdf::LambertianTransmission { bxdf_type, .. } => matches_flags(*bxdf_type, flag),
            Bxdf::OrenNayer { bxdf_type, .. } => matches_flags(*bxdf_type, flag),
            Bxdf::MicrofacetReflection { bxdf_type, .. } => matches_flags(*bxdf_type, flag),
            Bxdf::MicrofacetTransmission { bxdf_type, .. } => matches_flags(*bxdf_type, flag),
            Bxdf::FresnelBlend { bxdf_type, .. } => matches_flags(*bxdf_type, flag),
            Bxdf::FresnelSpecular { bxdf_type, .. } => matches_flags(*bxdf_type, flag),
            Bxdf::DisneyDiffuse { bxdf_type, .. } => matches_flags(*bxdf_type, flag),
            Bxdf::DisneyFakeSS { bxdf_type, .. } => matches_flags(*bxdf_type, flag),
            Bxdf::DisneyRetro { bxdf_type, .. } => matches_flags(*bxdf_type, flag),
            Bxdf::DisneySheen { bxdf_type, .. } => matches_flags(*bxdf_type, flag),
            Bxdf::DisneyClearCoat { bxdf_type, .. } => matches_flags(*bxdf_type, flag),
        }
    }

    /** This takes in incoming and outgoing directions of light, and returns the
        proportion of light coming from wi that goes in the direction of wo. (Output
        is a color)
    */
    pub fn f(bxdf: &Bxdf, wo: &Vector3<f64>, wi: &Vector3<f64>) -> Vector3<f64> {
        match bxdf {
            Bxdf::ScaledBxdf { inner, scale } => {
                Bxdf::f(inner.as_ref(), wo, wi).component_mul(scale)
            }
            // for arbitrary pair of directions, there will be no specular reflection (excluding the infinitesimal chance that the direction is right)
            Bxdf::SpecularReflection { .. } => util::black(),
            Bxdf::SpecularTransmission { .. } => util::black(),
            Bxdf::LambertianReflection { color, .. } => *color * INV_PI,
            Bxdf::LambertianTransmission { color, .. } => *color * INV_PI,
            Bxdf::OrenNayer {
                color,
                a,
                b,
                bxdf_type,
            } => {
                let sin_theta_i = sin_theta(wi);
                let sin_theta_o = sin_theta(wo);
                let mut max_cos: f64 = 0.;
                if sin_theta_i > SMALL && sin_theta_o > SMALL {
                    let sin_phi_i = sin_phi(wi);
                    let cos_phi_i = cos_phi(wi);
                    let sin_phi_o = sin_phi(wo);
                    let cos_phi_o = cos_phi(wo);
                    let d_cos = cos_phi_i * cos_phi_o + sin_phi_i * sin_phi_o;
                    max_cos = 0f64.max(d_cos);
                }
                let sin_alpha: f64;
                let tan_beta: f64;
                if abs_cos_theta(wi) > abs_cos_theta(wo) {
                    sin_alpha = sin_theta_o;
                    tan_beta = sin_theta_i / abs_cos_theta(wi);
                } else {
                    sin_alpha = sin_theta_i;
                    tan_beta = sin_theta_o / abs_cos_theta(wi);
                }
                color * (INV_PI * (a + b * max_cos * sin_alpha * tan_beta))
            }
            Bxdf::MicrofacetReflection {
                color,
                fresnel,
                mfd,
                bxdf_type,
            } => {
                let cos_theta_o = abs_cos_theta(wo);
                let cos_theta_i = abs_cos_theta(wi);
                let wh: Vector3<f64> = wi + wo; // half angle
                if cos_theta_i == 0. || cos_theta_o == 0. {
                    return util::black();
                }
                if wh == util::black() {
                    return util::black();
                }
                let wh = Unit::new_normalize(wh);
                // ensure wh is in the same hemisphere with faceforward
                let f = Fresnel::evaluate(
                    &fresnel,
                    wi.dot(&util::face_forward(wh, &Vector3::new(0f64, 0f64, 1f64))),
                );
                let comp1: Vector3<f64> = color
                    * MicrofacetDistribution::d(&mfd, &wh)
                    * MicrofacetDistribution::g(&mfd, wo, wi);
                let ans = comp1.component_mul(&f.scale(1f64 / (4f64 * cos_theta_i * cos_theta_o)));
                ans
            }
            Bxdf::MicrofacetTransmission {
                color,
                fresnel,
                mfd,
                eta_a,
                eta_b,
                mode,
                bxdf_type,
            } => {
                if util::same_hemisphere(wo, wi) {
                    // no transmission is possible
                    return util::black();
                }
                let cos_theta_o = cos_theta(wo);
                let cos_theta_i = cos_theta(wi);
                if cos_theta_i == 0. || cos_theta_o == 0. {
                    return util::black();
                }
                let eta = if cos_theta(wo) > 0. {
                    eta_b / eta_a
                } else {
                    eta_a / eta_b
                };
                let mut wh = Unit::new_normalize(wo + wi * eta);
                if wh.z < 0. {
                    wh = -wh;
                }

                if wo.dot(&wh) * wi.dot(&wh) > 0. {
                    // TODO: Is this redundant?
                    return util::black();
                }
                let f = Fresnel::evaluate(&fresnel, wo.dot(&wh));
                let sqrt_denom = wo.dot(&wh) + eta * wi.dot(&wh);
                let factor = if *mode == RADIANCE { 1. / eta } else { 1. };
                let c = (util::white() - f).component_mul(color);
                c.scale(
                    (MicrofacetDistribution::d(&mfd, &wh)
                        * MicrofacetDistribution::g(&mfd, wo, wi)
                        * eta
                        * eta
                        * wi.dot(&wh).abs()
                        * wo.dot(&wh).abs()
                        * factor
                        * factor
                        / (cos_theta_i * cos_theta_o * sqrt_denom * sqrt_denom))
                        .abs(),
                )
            }
            Bxdf::FresnelBlend {
                r_s,
                r_d,
                mfd,
                bxdf_type,
            } => {
                let diffuse: Vector3<f64> = (28. / (23. * PI)) * r_d;
                let diffuse = diffuse.component_mul(&(util::white() - r_s));
                let diffuse = diffuse.scale(
                    (1. - (1. - 0.5 * abs_cos_theta(wi)).powf(5.))
                        * (1. - (1. - 0.5 * abs_cos_theta(wo)).powf(5.)),
                );
                let wh: Vector3<f64> = wi + wo;
                if wh.abs().min() == 0. {
                    return util::black();
                }
                let wh = Unit::new_normalize(wh);
                let specular = MicrofacetDistribution::d(&mfd, &wh)
                    / (4. * wi.dot(&wh).abs() * abs_cos_theta(wi).max(abs_cos_theta(wo)))
                    * schlick_fresnel(r_s, wi.dot(&wh));
                diffuse + specular
            }
            Bxdf::FresnelSpecular { .. } => util::black(),
            Bxdf::DisneyDiffuse { color, .. } => {
                let fo = schlick_weight(abs_cos_theta(wo));
                let fi = schlick_weight(abs_cos_theta(wi));
                // formula from Burley 2015
                INV_PI * (1f64 - fo / 2f64) * (1f64 - fi / 2f64) * color
            }
            Bxdf::DisneyFakeSS {
                color, roughness, ..
            } => {
                let wh = wi + wo;
                if wh == util::black() {
                    return util::black();
                }
                let wh = Unit::new_normalize(wh);
                let cos_theta_d = wi.dot(&wh);

                let fss90 = cos_theta_d * cos_theta_d * roughness;
                let fo = schlick_weight(abs_cos_theta(wo));
                let fi = schlick_weight(abs_cos_theta(wi));
                let fss = util::lerp(fo, 1f64, fss90) * util::lerp(fi, 1f64, fss90);
                let ss =
                    1.25 * (fss * (1f64 / (abs_cos_theta(wo) + abs_cos_theta(wi)) - 0.5) + 0.5);
                INV_PI * ss * color
            }
            Bxdf::DisneyRetro {
                color, roughness, ..
            } => {
                let wh = wi + wo;
                if wh == util::black() {
                    return util::black();
                }
                let wh = Unit::new_normalize(wh);
                let cos_theta_d = wi.dot(&wh);
                let fo = schlick_weight(abs_cos_theta(wo));
                let fi = schlick_weight(abs_cos_theta(wi));
                let rr = 2f64 * roughness * cos_theta_d * cos_theta_d;
                // formula from Burley 2015
                INV_PI * rr * (fo + fi + fo * fi * (rr - 1f64)) * color
            }
            Bxdf::DisneySheen { color, .. } => {
                let wh = wi + wo;
                if wh == util::black() {
                    return util::black();
                }
                let wh = Unit::new_normalize(wh);
                let cos_theta_d = wi.dot(&wh);
                schlick_weight(cos_theta_d) * color
            }
            Bxdf::DisneyClearCoat { weight, gloss, .. } => {
                let wh = wi + wo;
                if wh == util::black() {
                    return util::black();
                }
                let wh = Unit::new_normalize(wh);
                // assumes IOR = 1.5 so that R0 = 0.04, saves call to eta_to_r0
                let dr = gtr1(abs_cos_theta(&wh), *gloss);
                let fr = fr_schlick(&util::white().scale(0.04), wo.dot(&wh));
                let gr = smithg_ggx(abs_cos_theta(wo), 0.25) * smithg_ggx(abs_cos_theta(wi), 0.25);

                weight / 4f64 * gr * dr * fr // return gray color
            }
        }
    }

    /** Order of outputs is:
    color, vector, pdf
    */
    pub fn sample_f(
        bxdf: &Bxdf,
        wo: &Vector3<f64>,
        sample: &Point2<f64>,
        sample_type: u8,
    ) -> (Vector3<f64>, Vector3<f64>, f64) {
        match bxdf {
            Bxdf::ScaledBxdf { inner, scale } => {
                let (color, wi, pdf) = Bxdf::sample_f(inner.as_ref(), wo, sample, sample_type);
                (color.component_mul(scale), wi, pdf)
            }
            Bxdf::SpecularReflection {
                color,
                fresnel,
                bxdf_type,
            } => {
                // calculate perfectly specular reflection (around specific coordinate system)
                let wi = Vector3::new(-wo.x, -wo.y, wo.z);
                let out_color = color.component_mul(&(Fresnel::evaluate(&fresnel, cos_theta(&wi))));
                (out_color, wi, 1.) // 1. is pdf
            }
            Bxdf::SpecularTransmission {
                color,
                eta_a,
                eta_b,
                fresnel,
                mode,
                bxdf_type,
            } => {
                let entering = cos_theta(wo) > 0.;
                let eta_i = if entering { *eta_a } else { *eta_b };
                let eta_t = if entering { *eta_b } else { *eta_a };
                // compute refraction
                let dir = util::refract(
                    &wo,
                    &util::face_forward(Unit::new_normalize(Vector3::new(0., 0., 1.)), &wo),
                    eta_i / eta_t,
                );
                match dir {
                    Some(vec) => {
                        let mut ft: Vector3<f64> =
                            util::white() - Fresnel::evaluate(fresnel, cos_theta(&vec));
                        ft.component_mul_assign(color);
                        if *mode == RADIANCE {
                            ft = ft.scale((eta_i * eta_i) / (eta_t * eta_t));
                        }
                        (ft, vec, 1.)
                    }
                    None => (util::black(), Vector3::new(0., 0., 0.), 0.), // TODO: change this?
                }
            }
            Bxdf::LambertianReflection { .. } => {
                Bxdf::default_sample_f(bxdf, wo, sample, sample_type)
            }
            Bxdf::OrenNayer { .. } => Bxdf::default_sample_f(bxdf, wo, sample, sample_type),
            Bxdf::MicrofacetReflection {
                color,
                fresnel,
                mfd,
                bxdf_type,
            } => {
                // Bxdf::default_sample_f(bxdf, wo, sample, sample_type)
                if wo.z == 0. {
                    return (util::black(), util::black(), 0.);
                }
                let wh = MicrofacetDistribution::sample_wh(mfd, &wo, sample);
                if wo.dot(&wh) < 0. {
                    (util::black(), util::black(), 0.);
                }
                let wi = util::reflect(wo, &wh);
                if !util::same_hemisphere(wo, &wi) {
                    return (util::black(), util::black(), 0.);
                }
                let pdf = MicrofacetDistribution::pdf(mfd, wo, &wh) / (4. * wo.dot(&wh));
                (Bxdf::f(bxdf, wo, &wi), wi, pdf)
            }
            Bxdf::MicrofacetTransmission {
                color,
                fresnel,
                mfd,
                eta_a,
                eta_b,
                mode,
                bxdf_type,
            } => {
                if wo.z == 0. {
                    return (util::black(), util::black(), 0.);
                }
                let wh = MicrofacetDistribution::sample_wh(mfd, &wo, sample);
                if wo.dot(&wh) < 0. {
                    return (util::black(), util::black(), 0.);
                }
                let eta = if cos_theta(wo) > 0. {
                    eta_a / eta_b
                } else {
                    eta_b / eta_a
                };
                let transmission = util::refract(&wo, &wh, eta);
                match transmission {
                    Some(wi) => {
                        let pdf = Bxdf::pdf(bxdf, wo, &wi);
                        let color = Bxdf::f(bxdf, wo, &wi);
                        (color, wi, pdf)
                    }
                    None => (util::black(), util::black(), 0.),
                }
            }
            Bxdf::FresnelBlend { .. } => Bxdf::default_sample_f(bxdf, wo, sample, sample_type),
            Bxdf::FresnelSpecular {
                r,
                t,
                eta_a,
                eta_b,
                mode,
                bxdf_type,
            } => {
                let f = fr_dielectric(cos_theta(wo) / wo.magnitude(), *eta_a, *eta_b);
                if sample[0] < f {
                    // perfect specular reflectance
                    if sample_type != 0u8 {
                        // TODO: something?
                    }
                    let wi = Vector3::new(-wo.x, -wo.y, wo.z);
                    let pdf = f;
                    let v = r.scale(f);
                    return (v, wi, pdf);
                } else {
                    let entering = cos_theta(wo) > 0.;
                    let eta_i = if entering { eta_a } else { eta_b };
                    let eta_t = if entering { eta_b } else { eta_a };
                    let dir = util::refract(
                        &wo,
                        &util::face_forward(Unit::new_normalize(Vector3::new(0., 0., 1.)), &wo),
                        eta_i / eta_t,
                    );
                    match dir {
                        Some(direction) => {
                            let mut ft: Vector3<f64> = t.scale(1. - f);
                            if *mode == RADIANCE {
                                ft *= (eta_i * eta_i) / (eta_t * eta_t);
                            }
                            if sample_type != 0u8 {
                                // TODO: something?
                            }
                            let pdf = 1. - f;
                            return (ft, direction, pdf);
                        }
                        None => {
                            return (util::black(), util::black(), 0.);
                        }
                    }
                }
            }
            Bxdf::DisneyDiffuse { .. } => Bxdf::default_sample_f(bxdf, wo, sample, sample_type),
            Bxdf::DisneyFakeSS { .. } => Bxdf::default_sample_f(bxdf, wo, sample, sample_type),
            Bxdf::DisneyRetro { .. } => Bxdf::default_sample_f(bxdf, wo, sample, sample_type),
            Bxdf::DisneySheen { .. } => Bxdf::default_sample_f(bxdf, wo, sample, sample_type),
            Bxdf::DisneyClearCoat { gloss, .. } => {
                if wo.z == 0f64 {
                    return (util::black(), util::black(), 0f64);
                }
                let alpha2 = gloss * gloss;
                let cos_theta = 0f64
                    .max((1f64 - alpha2.powf(1f64 - sample[0])) / (1f64 - alpha2))
                    .sqrt();
                let sin_theta = 0f64.max(1f64 - cos_theta * cos_theta).sqrt();
                let phi = 2f64 * PI * sample[1];
                let mut wh = util::make_spherical(sin_theta, cos_theta, phi);
                if !util::same_hemisphere(wo, &wh) {
                    wh = -wh;
                }
                let wi = util::reflect(wo, &Unit::new_normalize(wh));
                if !util::same_hemisphere(wo, &wi) {
                    return (util::black(), util::black(), 0f64);
                }
                (Bxdf::f(bxdf, wo, &wi), wi, Bxdf::pdf(bxdf, wo, &wi))
            }
            Bxdf::LambertianTransmission { color, bxdf_type } => {
                let mut wi = util::cosine_sample_hemisphere(sample);
                if wi.z > 0f64 {
                    // ensure it's in the opposite hemisphere
                    wi.z = -wi.z;
                }
                let pdf = Bxdf::pdf(bxdf, wo, &wi);
                (Bxdf::f(bxdf, wo, &wi), wi, pdf)
            }
        }
    }

    pub fn pdf(&self, wo: &Vector3<f64>, wi: &Vector3<f64>) -> f64 {
        match self {
            Bxdf::ScaledBxdf { inner, .. } => Bxdf::pdf(inner.as_ref(), wo, wi),
            Bxdf::SpecularReflection { .. } => Bxdf::default_pdf(wo, wi),
            Bxdf::SpecularTransmission { .. } => 0f64,
            Bxdf::LambertianReflection { .. } => Bxdf::default_pdf(wo, wi),
            Bxdf::LambertianTransmission { .. } => {
                if !util::same_hemisphere(wo, wi) {
                    abs_cos_theta(wi) * INV_PI
                } else {
                    0.
                }
            }
            Bxdf::OrenNayer { .. } => Bxdf::default_pdf(wo, wi),
            Bxdf::MicrofacetReflection { mfd, .. } => {
                if !util::same_hemisphere(wo, wi) {
                    return 0.;
                }
                let wh: Unit<Vector3<f64>> = Unit::new_normalize(wo + wi);
                MicrofacetDistribution::pdf(&mfd, wo, &wh) / (4. * wo.dot(&wh))
            }
            Bxdf::MicrofacetTransmission {
                mfd, eta_a, eta_b, ..
            } => {
                if util::same_hemisphere(wo, wi) {
                    return 0.;
                }
                let eta = if cos_theta(wo) > 0. {
                    eta_b / eta_a
                } else {
                    eta_a / eta_b
                };
                let wh: Unit<Vector3<f64>> = Unit::new_normalize(wo + wi * eta);
                if wo.dot(&wh) * wi.dot(&wh) > 0. {
                    return 0.;
                }
                let sqrt_denom = wo.dot(&wh) + eta * wi.dot(&wh);
                // jacobian for change of variables from macrosurface to microsurface
                let dwh_dwi = (eta * eta * wi.dot(&wh)).abs() / (sqrt_denom * sqrt_denom);
                let ans = MicrofacetDistribution::pdf(mfd, wo, &wh);

                MicrofacetDistribution::pdf(mfd, wo, &wh) * dwh_dwi
            }
            Bxdf::FresnelBlend {
                r_s,
                r_d,
                mfd,
                bxdf_type,
            } => {
                println!(
                    "Calling unimplemented function: line {} in {}",
                    line!(),
                    file!()
                );
                0.
            }
            Bxdf::FresnelSpecular {
                r,
                t,
                eta_a,
                eta_b,
                mode,
                bxdf_type,
            } => {
                println!(
                    "Error: Calling unimplemented function on line {} in {}.",
                    line!(),
                    file!()
                );
                0.
            }
            Bxdf::DisneyDiffuse { .. } => Bxdf::default_pdf(wo, wi),
            Bxdf::DisneyFakeSS { .. } => Bxdf::default_pdf(wo, wi),
            Bxdf::DisneyRetro { .. } => Bxdf::default_pdf(wo, wi),
            Bxdf::DisneySheen { .. } => Bxdf::default_pdf(wo, wi),
            Bxdf::DisneyClearCoat {
                weight,
                gloss,
                bxdf_type,
            } => {
                if !util::same_hemisphere(wo, wi) {
                    return 0f64;
                }
                let wh = wi + wo;
                if wh == util::black() {
                    return 0f64;
                }
                let wh = Unit::new_normalize(wh);
                let dr = gtr1(abs_cos_theta(&wh), *gloss);
                dr * abs_cos_theta(&wh) / (4f64 * wo.dot(&wh))
            }
        }
    }

    pub fn default_sample_f(
        bxdf: &Bxdf,
        wo: &Vector3<f64>,
        sample: &Point2<f64>,
        sample_type: u8,
    ) -> (Vector3<f64>, Vector3<f64>, f64) {
        let mut wi = util::rand_cosine_dir();
        if wo.z < 0. {
            wi.z *= -1.;
        }
        let pdf = Bxdf::pdf(bxdf, wo, &wi);
        (Bxdf::f(bxdf, wo, &wi), wi, pdf)
    }

    pub fn default_pdf(wo: &Vector3<f64>, wi: &Vector3<f64>) -> f64 {
        if util::same_hemisphere(wo, wi) {
            abs_cos_theta(wi) * INV_PI
        } else {
            0.
        }
    }

    pub fn get_type(&self) -> u8 {
        match self {
            Bxdf::ScaledBxdf { inner, .. } => Bxdf::get_type(inner.as_ref()),
            Bxdf::SpecularReflection { bxdf_type, .. } => *bxdf_type,
            Bxdf::SpecularTransmission { bxdf_type, .. } => *bxdf_type,
            Bxdf::LambertianReflection { bxdf_type, .. } => *bxdf_type,
            Bxdf::LambertianTransmission { bxdf_type, .. } => *bxdf_type,
            Bxdf::OrenNayer { bxdf_type, .. } => *bxdf_type,
            Bxdf::MicrofacetReflection { bxdf_type, .. } => *bxdf_type,
            Bxdf::MicrofacetTransmission { bxdf_type, .. } => *bxdf_type,
            Bxdf::FresnelBlend { bxdf_type, .. } => *bxdf_type,
            Bxdf::FresnelSpecular { bxdf_type, .. } => *bxdf_type,
            Bxdf::DisneyDiffuse { bxdf_type, .. } => *bxdf_type,
            Bxdf::DisneyFakeSS { bxdf_type, .. } => *bxdf_type,
            Bxdf::DisneyRetro { bxdf_type, .. } => *bxdf_type,
            Bxdf::DisneySheen { bxdf_type, .. } => *bxdf_type,
            Bxdf::DisneyClearCoat { bxdf_type, .. } => *bxdf_type,
        }
    }

    #[allow(dead_code)]
    pub fn get_name(&self) -> &str {
        match self {
            Bxdf::ScaledBxdf { .. } => "Scaled",
            Bxdf::SpecularReflection { .. } => "Specular Reflection",
            Bxdf::SpecularTransmission { .. } => "Specular Transmission",
            Bxdf::LambertianReflection { .. } => "Lambertian Reflection",
            Bxdf::LambertianTransmission { .. } => "Lambertian Transmission",
            Bxdf::OrenNayer { .. } => "Oren Nayer",
            Bxdf::MicrofacetReflection { .. } => "Microfacet Refl",
            Bxdf::MicrofacetTransmission { .. } => "Microfacet Transmission",
            Bxdf::FresnelBlend { .. } => "Fresnel Blend",
            Bxdf::FresnelSpecular { .. } => "Fresnel Specular",
            Bxdf::DisneyDiffuse { .. } => "Disney Diffuse",
            Bxdf::DisneyFakeSS { .. } => "Disney Fake Subsurface",
            Bxdf::DisneyRetro { .. } => "Disney Retroreflection",
            Bxdf::DisneySheen { .. } => "Disney Sheen",
            Bxdf::DisneyClearCoat { .. } => "Disney Clear Coat",
        }
    }

    pub fn make_scaled(inner: Bxdf, scale: Vector3<f64>) -> Self {
        Bxdf::ScaledBxdf {
            inner: Box::new(inner),
            scale,
        }
    }

    pub fn make_specular_reflection(color: Vector3<f64>, fresnel: Fresnel) -> Self {
        Bxdf::SpecularReflection {
            fresnel,
            color,
            bxdf_type: BSDF_REFLECTION | BSDF_SPECULAR,
        }
    }

    pub fn make_specular_transmission(
        color: Vector3<f64>,
        eta_a: f64,
        eta_b: f64,
        mode: u8,
    ) -> Self {
        Bxdf::SpecularTransmission {
            color,
            fresnel: Fresnel::FresnelDielectric {
                eta_i: eta_a,
                eta_t: eta_b,
            },
            mode,
            eta_a,
            eta_b,
            bxdf_type: BSDF_TRANSMISSION | BSDF_SPECULAR,
        }
    }

    pub fn make_lambertian_reflection(color: Vector3<f64>) -> Self {
        let bxdf_type = BSDF_REFLECTION | BSDF_DIFFUSE;
        Bxdf::LambertianReflection { color, bxdf_type }
    }

    pub fn make_oren_nayar(color: Vector3<f64>, sigma: f64) -> Self {
        let bxdf_type = BSDF_REFLECTION | BSDF_DIFFUSE;
        let sigma_r = sigma * PI / 180.0;
        let sigma2 = sigma_r * sigma_r;
        let a = 1f64 - (sigma2 / (2f64 * (sigma2 + 0.33)));
        let b = 0.45 * sigma2 / (sigma2 + 0.09);
        Bxdf::OrenNayer {
            color,
            a,
            b,
            bxdf_type,
        }
    }

    pub fn make_microfacet_reflection(
        color: Vector3<f64>,
        fresnel: Fresnel,
        mfd: MicrofacetDistribution,
    ) -> Self {
        let bxdf_type = BSDF_REFLECTION | BSDF_GLOSSY;
        Bxdf::MicrofacetReflection {
            color,
            fresnel,
            mfd,
            bxdf_type,
        }
    }

    pub fn make_microfacet_transmission(
        color: Vector3<f64>,
        mfd: MicrofacetDistribution,
        eta_a: f64,
        eta_b: f64,
        mode: u8,
    ) -> Self {
        let bxdf_type = BSDF_TRANSMISSION | BSDF_GLOSSY;
        let fresnel = Fresnel::FresnelDielectric {
            eta_i: eta_a,
            eta_t: eta_b,
        };
        Bxdf::MicrofacetTransmission {
            color,
            fresnel,
            mfd,
            eta_a,
            eta_b,
            mode,
            bxdf_type,
        }
    }

    pub fn make_fresnel_blend(
        r_d: Vector3<f64>,
        r_s: Vector3<f64>,
        mfd: MicrofacetDistribution,
    ) -> Self {
        let bxdf_type = BSDF_REFLECTION | BSDF_GLOSSY;
        Bxdf::FresnelBlend {
            r_d,
            r_s,
            mfd,
            bxdf_type,
        }
    }

    pub fn make_fresnel_specular(
        r: Vector3<f64>,
        t: Vector3<f64>,
        eta_a: f64,
        eta_b: f64,
        mode: u8,
    ) -> Self {
        let bxdf_type = BSDF_TRANSMISSION | BSDF_REFLECTION | BSDF_SPECULAR;
        Bxdf::FresnelSpecular {
            r,
            t,
            eta_a,
            eta_b,
            mode,
            bxdf_type,
        }
    }

    pub fn make_lambertian_transmission(color: Vector3<f64>) -> Self {
        Bxdf::LambertianTransmission {
            color,
            bxdf_type: BSDF_TRANSMISSION | BSDF_DIFFUSE,
        }
    }

    pub fn make_disney_diffuse(color: Vector3<f64>) -> Self {
        let bxdf_type = BSDF_DIFFUSE | BSDF_REFLECTION;
        Bxdf::DisneyDiffuse { color, bxdf_type }
    }

    pub fn make_disney_fake_ss(color: Vector3<f64>, roughness: f64) -> Self {
        let bxdf_type = BSDF_DIFFUSE | BSDF_REFLECTION;
        Bxdf::DisneyFakeSS {
            color,
            bxdf_type,
            roughness,
        }
    }

    pub fn make_disney_retro(color: Vector3<f64>, roughness: f64) -> Self {
        let bxdf_type = BSDF_DIFFUSE | BSDF_REFLECTION;
        Bxdf::DisneyRetro {
            color,
            bxdf_type,
            roughness,
        }
    }

    pub fn make_disney_sheen(color: Vector3<f64>) -> Self {
        let bxdf_type = BSDF_DIFFUSE | BSDF_REFLECTION;
        Bxdf::DisneySheen { color, bxdf_type }
    }

    pub fn make_disney_clearcoat(weight: f64, gloss: f64) -> Self {
        let bxdf_type = BSDF_GLOSSY | BSDF_REFLECTION;
        Bxdf::DisneyClearCoat {
            weight,
            gloss,
            bxdf_type,
        }
    }
}
