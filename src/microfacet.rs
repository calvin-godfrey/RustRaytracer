#![allow(dead_code)]
use crate::bxdf;
use crate::consts::*;
use crate::util;
use nalgebra::{
    base::{Unit, Vector3},
    geometry::Point2,
};

#[derive(Copy, Clone)]
pub enum MicrofacetDistribution {
    Beckmann {
        sample_visible_area: bool,
        alpha_x: f64,
        alpha_y: f64,
    },
    TrowbridgeReitz {
        sample_visible_area: bool,
        alpha_x: f64,
        alpha_y: f64,
    },
    DisneyMicrofacet {
        sample_visible_area: bool,
        alpha_x: f64,
        alpha_y: f64,
    },
    Empty,
}

#[allow(unused_variables)]
impl MicrofacetDistribution {
    /** This gives the differential area of microfacets with the given surface normal w_h.
     */
    pub fn d(mfd: &MicrofacetDistribution, wh: &Vector3<f64>) -> f64 {
        match mfd {
            MicrofacetDistribution::Beckmann {
                sample_visible_area,
                alpha_x,
                alpha_y,
            } => {
                let tan_sq_theta = bxdf::tan_sq_theta(wh);
                if tan_sq_theta == std::f64::INFINITY {
                    return 0.;
                }
                let cos4_theta = bxdf::cos_sq_theta(wh) * bxdf::cos_sq_theta(wh);
                // e^{-\tan^2\theta_h/\alpha^2} / (\pi \alpha^2 \cos^4\theta_h)
                (-tan_sq_theta
                    * (bxdf::cos_sq_phi(wh) / (alpha_x * alpha_x)
                        + bxdf::sin_sq_phi(wh) / (alpha_y * alpha_y)))
                    .exp()
                    / (PI * alpha_x * alpha_y * cos4_theta)
            }
            MicrofacetDistribution::TrowbridgeReitz {
                sample_visible_area,
                alpha_x,
                alpha_y,
            } => {
                let tan_sq_theta = bxdf::tan_sq_theta(wh);
                if tan_sq_theta == std::f64::INFINITY {
                    return 0.;
                }
                let cos4_theta = bxdf::cos_sq_theta(wh) * bxdf::cos_sq_theta(wh);
                let e = (bxdf::cos_sq_phi(wh) / (alpha_x * alpha_x)
                    + bxdf::sin_sq_phi(wh) / (alpha_y * alpha_y))
                    * tan_sq_theta;
                // 1 / (\pi \alpha_x \alpha_y\cos^4\theta_h(1 + \tan^2\theta_h(\cos^2\phi_h/\alpha_x^2+\sin^2\phi_h/\alpha_y^2))^2)
                1f64 / (PI * alpha_x * alpha_y * cos4_theta * (1. + e) * (1. + e))
            }
            MicrofacetDistribution::DisneyMicrofacet {
                sample_visible_area,
                alpha_x,
                alpha_y,
            } => {
                let tan_sq_theta = bxdf::tan_sq_theta(wh);
                if tan_sq_theta == std::f64::INFINITY {
                    return 0.;
                }
                let cos4_theta = bxdf::cos_sq_theta(wh) * bxdf::cos_sq_theta(wh);
                let e = (bxdf::cos_sq_phi(wh) / (alpha_x * alpha_x)
                    + bxdf::sin_sq_phi(wh) / (alpha_y * alpha_y))
                    * tan_sq_theta;
                // 1 / (\pi \alpha_x \alpha_y\cos^4\theta_h(1 + \tan^2\theta_h(\cos^2\phi_h/\alpha_x^2+\sin^2\phi_h/\alpha_y^2))^2)
                1f64 / (PI * alpha_x * alpha_y * cos4_theta * (1. + e) * (1. + e))
            }
            MicrofacetDistribution::Empty => 0.,
        }
    }

    pub fn lambda(mfd: &MicrofacetDistribution, w: &Vector3<f64>) -> f64 {
        match mfd {
            MicrofacetDistribution::Beckmann {
                sample_visible_area,
                alpha_x,
                alpha_y,
            } => {
                let abs_tan_theta = bxdf::tan_theta(w).abs();
                if abs_tan_theta == std::f64::INFINITY {
                    return 0.;
                }
                let alpha = (bxdf::cos_sq_phi(w) * alpha_x * alpha_x
                    + bxdf::sin_sq_phi(w) * alpha_y * alpha_y)
                    .sqrt();
                let a = 1. / (alpha * abs_tan_theta);
                if a >= 1.6 {
                    return 0.;
                }
                (1. - 1.259 * a + 0.396 * a * a) / (3.535 * a + 2.181 * a * a)
            }
            MicrofacetDistribution::TrowbridgeReitz {
                sample_visible_area,
                alpha_x,
                alpha_y,
            } => {
                let abs_tan_theta = bxdf::tan_theta(w).abs();
                if abs_tan_theta == std::f64::INFINITY {
                    return 0.;
                }
                let alpha = (bxdf::cos_sq_phi(w) * alpha_x * alpha_x
                    + bxdf::sin_sq_phi(w) * alpha_y * alpha_y)
                    .sqrt();
                let alpha2_ta2_theta = (alpha * abs_tan_theta) * (alpha * abs_tan_theta);
                (-1f64 + (1f64 + alpha2_ta2_theta).sqrt()) / 2f64
            }
            MicrofacetDistribution::DisneyMicrofacet {
                sample_visible_area,
                alpha_x,
                alpha_y,
            } => {
                let abs_tan_theta = bxdf::tan_theta(w).abs();
                if abs_tan_theta == std::f64::INFINITY {
                    return 0.;
                }
                let alpha = (bxdf::cos_sq_phi(w) * alpha_x * alpha_x
                    + bxdf::sin_sq_phi(w) * alpha_y * alpha_y)
                    .sqrt();
                let alpha2_ta2_theta = (alpha * abs_tan_theta) * (alpha * abs_tan_theta);
                (-1f64 + (1f64 + alpha2_ta2_theta).sqrt()) / 2f64
            }
            MicrofacetDistribution::Empty => 0.,
        }
    }

    pub fn g1(mfd: &MicrofacetDistribution, w: &Vector3<f64>) -> f64 {
        1. / (1. + MicrofacetDistribution::lambda(mfd, w))
    }
    pub fn g(mfd: &MicrofacetDistribution, wo: &Vector3<f64>, wi: &Vector3<f64>) -> f64 {
        match mfd {
            MicrofacetDistribution::DisneyMicrofacet { .. } => {
                MicrofacetDistribution::g1(mfd, wo) * MicrofacetDistribution::g1(mfd, wi)
            }
            _ => {
                1. / (1.
                    + MicrofacetDistribution::lambda(mfd, wo)
                    + MicrofacetDistribution::lambda(mfd, wi))
            }
        }
    }

    pub fn pdf(mfd: &MicrofacetDistribution, wo: &Vector3<f64>, wh: &Vector3<f64>) -> f64 {
        MicrofacetDistribution::default_pdf(mfd, wo, wh)
    }

    fn default_pdf(mfd: &MicrofacetDistribution, wo: &Vector3<f64>, wh: &Vector3<f64>) -> f64 {
        if MicrofacetDistribution::get_visible_area(mfd) {
            return MicrofacetDistribution::d(mfd, wh)
                * MicrofacetDistribution::g1(mfd, wo)
                * wo.dot(wh).abs()
                / bxdf::abs_cos_theta(wo);
        } else {
            return MicrofacetDistribution::d(mfd, wh) * bxdf::abs_cos_theta(wh);
        }
    }
    fn get_visible_area(mfd: &MicrofacetDistribution) -> bool {
        match mfd {
            MicrofacetDistribution::Beckmann {
                sample_visible_area,
                ..
            } => *sample_visible_area,
            MicrofacetDistribution::TrowbridgeReitz {
                sample_visible_area,
                ..
            } => *sample_visible_area,
            MicrofacetDistribution::DisneyMicrofacet {
                sample_visible_area,
                ..
            } => *sample_visible_area,
            MicrofacetDistribution::Empty => false,
        }
    }

    pub fn sample_wh(
        mfd: &MicrofacetDistribution,
        wo: &Vector3<f64>,
        u: &Point2<f64>,
    ) -> Unit<Vector3<f64>> {
        match mfd {
            MicrofacetDistribution::Beckmann {
                sample_visible_area,
                alpha_x,
                alpha_y,
            } => {
                return if !sample_visible_area {
                    let tan_2_theta: f64;
                    let mut phi: f64;
                    if alpha_x == alpha_y {
                        let log_sample: f64 = (1. - u[0]).ln();
                        tan_2_theta = -alpha_x * alpha_x * log_sample;
                        phi = u[1] * 2. * PI;
                    } else {
                        let log_sample = (1. - u[0]).ln();
                        phi = (alpha_y / alpha_x * (2. * PI * u[1] + 0.5 * PI).tan()).atan();
                        if u[1] > 0.5 {
                            phi += PI;
                        }
                        let sin_phi = phi.sin();
                        let cos_phi = phi.cos();
                        let alpha_x2 = alpha_x * alpha_x;
                        let alpha_y2 = alpha_y * alpha_y;
                        tan_2_theta = -log_sample
                            / (cos_phi * cos_phi / alpha_x2 + sin_phi * sin_phi / alpha_y2);
                    }
                    let cos_theta = 1. / (1. + tan_2_theta).sqrt();
                    let sin_theta = (0f64.max(1. - cos_theta * cos_theta)).sqrt();
                    let mut wh = util::make_spherical(sin_theta, cos_theta, phi);
                    if !util::same_hemisphere(wo, &wh) {
                        wh = -wh;
                    }
                    Unit::new_normalize(wh)
                } else {
                    let flip = wo.z < 0.;
                    let flipped_wo = if flip { -wo } else { *wo };
                    let wh = beckmann_sample(&flipped_wo, *alpha_x, *alpha_y, u[0], u[1]);
                    if flip {
                        -wh
                    } else {
                        wh
                    }
                }
            }
            MicrofacetDistribution::TrowbridgeReitz {
                sample_visible_area,
                alpha_x,
                alpha_y,
            } => {
                return if !sample_visible_area {
                    let cos_theta: f64;
                    let mut phi = 2.0 * PI * u[1];
                    if alpha_x == alpha_y {
                        let tan_theta2 = alpha_x * alpha_x * u[0] / (1. - u[0]);
                        cos_theta = 1. / ((1. + tan_theta2).sqrt());
                    } else {
                        phi = (alpha_y / alpha_x * (2. * PI * u[1] + 0.5 * PI).tan()).atan();
                        if u[1] > 0.5 {
                            phi += PI;
                        }
                        let sin_phi = phi.sin();
                        let cos_phi = phi.cos();
                        let alphax2 = alpha_x * alpha_x;
                        let alphay2 = alpha_y * alpha_y;
                        let alpha2 =
                            1.0 / (cos_phi * cos_phi / alphax2 + sin_phi * sin_phi / alphay2);
                        let tan_theta2 = alpha2 * u[0] / (1. - u[0]);

                        cos_theta = 1. / (1. + tan_theta2).sqrt();
                    }
                    let sin_theta = (0f64.max(1. - cos_theta * cos_theta)).sqrt();
                    let mut wh = util::make_spherical(sin_theta, cos_theta, phi);
                    if !util::same_hemisphere(wo, &wh) {
                        wh = -wh;
                    }
                    Unit::new_normalize(wh)
                } else {
                    let flip = wo.z < 0.;
                    let flipped_wo = if flip { -wo } else { *wo };
                    let wh = trowbridge_reitz_sample(&flipped_wo, *alpha_x, *alpha_y, u[0], u[1]);
                    if flip {
                        -wh
                    } else {
                        wh
                    }
                }
            }
            MicrofacetDistribution::DisneyMicrofacet {
                sample_visible_area,
                alpha_x,
                alpha_y,
            } => {
                return if !sample_visible_area {
                    let cos_theta: f64;
                    let mut phi = 2.0 * PI * u[1];
                    if alpha_x == alpha_y {
                        let tan_theta2 = alpha_x * alpha_x * u[0] / (1. - u[0]);
                        cos_theta = 1. / ((1. + tan_theta2).sqrt());
                    } else {
                        phi = (alpha_y / alpha_x * (2. * PI * u[1] + 0.5 * PI).tan()).atan();
                        if u[1] > 0.5 {
                            phi += PI;
                        }
                        let sin_phi = phi.sin();
                        let cos_phi = phi.cos();
                        let alphax2 = alpha_x * alpha_x;
                        let alphay2 = alpha_y * alpha_y;
                        let alpha2 =
                            1.0 / (cos_phi * cos_phi / alphax2 + sin_phi * sin_phi / alphay2);
                        let tan_theta2 = alpha2 * u[0] / (1. - u[0]);

                        cos_theta = 1. / (1. + tan_theta2).sqrt();
                    }
                    let sin_theta = (0f64.max(1. - cos_theta * cos_theta)).sqrt();
                    let mut wh = util::make_spherical(sin_theta, cos_theta, phi);
                    if !util::same_hemisphere(wo, &wh) {
                        wh = -wh;
                    }
                    Unit::new_normalize(wh)
                } else {
                    let flip = wo.z < 0.;
                    let flipped_wo = if flip { -wo } else { *wo };
                    let wh = trowbridge_reitz_sample(&flipped_wo, *alpha_x, *alpha_y, u[0], u[1]);
                    if flip {
                        -wh
                    } else {
                        wh
                    }
                }
            }
            MicrofacetDistribution::Empty => Unit::new_normalize(util::black()),
        }
    }

    pub fn make_beckmann(alpha_x: f64, alpha_y: f64, samplevis: bool) -> Self {
        let x = alpha_x.max(1e-3);
        let y = alpha_y.max(1e-3);
        MicrofacetDistribution::Beckmann {
            sample_visible_area: samplevis,
            alpha_x: x,
            alpha_y: y,
        }
    }

    pub fn make_trowbridge_reitz(alpha_x: f64, alpha_y: f64, samplevis: bool) -> Self {
        let x = alpha_x.max(1e-3);
        let y = alpha_y.max(1e-3);
        MicrofacetDistribution::TrowbridgeReitz {
            sample_visible_area: samplevis,
            alpha_x: x,
            alpha_y: y,
        }
    }

    pub fn make_disney(alpha_x: f64, alpha_y: f64) -> Self {
        let x = alpha_x.max(1e-3);
        let y = alpha_y.max(1e-3);
        MicrofacetDistribution::DisneyMicrofacet {
            sample_visible_area: true,
            alpha_x: x,
            alpha_y: y,
        }
    }
}

pub fn beckmann_roughness_to_alpha(roughness: f64) -> f64 {
    let roughness = roughness.max(1e-3);
    let x = roughness.ln();
    1.62142 + 0.819955 * x + 0.1734 * x * x + 0.0171201 * x * x * x + 0.000640711 * x * x * x * x
}

fn beckmann_sample(
    wi: &Vector3<f64>,
    alpha_x: f64,
    alpha_y: f64,
    u1: f64,
    u2: f64,
) -> Unit<Vector3<f64>> {
    let wi_s = Unit::new_normalize(Vector3::new(alpha_x * wi.x, alpha_y * wi.y, wi.z));
    let slope = beckmann_sample_11(bxdf::cos_theta(&wi_s), u1, u2);
    let mut slope_x = slope.x;
    let mut slope_y = slope.y;
    let wi_s_sin_phi = bxdf::sin_phi(&wi_s);
    let wi_s_cos_phi = bxdf::cos_phi(&wi_s);
    // rotate
    let tmp = wi_s_cos_phi * slope_x - wi_s_sin_phi * slope_y;
    slope_y = wi_s_sin_phi * slope_x + wi_s_cos_phi * slope_y;
    slope_x = tmp;

    slope_x = alpha_x * slope_x;
    slope_y = alpha_y * slope_y;
    Unit::new_normalize(Vector3::new(-slope_x, -slope_y, 1.))
}

fn beckmann_sample_11(cos_theta_i: f64, u1: f64, u2: f64) -> Point2<f64> {
    if cos_theta_i > 0.9999 {
        let r = (-(1. - u1).ln()).sqrt();
        let sin_phi = (2. * PI * u2).sin();
        let cos_phi = (2. * PI * u2).cos();
        return Point2::new(r * cos_phi, r * sin_phi);
    }

    let sin_theta_i = (0f64.max(1. - cos_theta_i * cos_theta_i)).sqrt();
    let tan_theta_i = sin_theta_i / cos_theta_i;
    let cot_theta_i = 1. / tan_theta_i;

    let mut a = -1.;
    let mut c = util::erf(cos_theta_i);
    let sample_x = u1.max(1e-6);
    let theta_i = cos_theta_i.acos();
    let fit = 1. + theta_i * (-0.876 + theta_i * (0.4265 - 0.0594 * theta_i));
    let mut b = c - (1. + c) * (1. - sample_x).powf(fit);

    const SQRT_INV_PI: f64 = std::f64::consts::FRAC_1_PI;
    let normalization =
        1. / (1. + c + SQRT_INV_PI * tan_theta_i * (-cot_theta_i * cot_theta_i).exp());

    let mut it = 0;

    while it + 1 < 10 {
        it = it + 1;
        if !(b >= a && b <= c) {
            b = 0.5 * (a + c);
        }

        let inv_erf = util::inv_erf(b);
        let value = normalization
            * (1. + b + SQRT_INV_PI * tan_theta_i * (-inv_erf * inv_erf).exp())
            - sample_x;
        let derivative = normalization * (1. - inv_erf * tan_theta_i);

        if value.abs() < 1e-5 {
            break;
        }
        if value > 0. {
            c = b;
        } else {
            a = b;
        }
        b = b - value / derivative;
    }
    let slope_x = util::inv_erf(b);
    let slope_y = util::inv_erf(2. * u2.max(1e-6) - 1.);
    return Point2::new(slope_x, slope_y);
}

pub fn trowbridge_reitz_roughness_to_alpha(roughness: f64) -> f64 {
    let roughness = roughness.max(1e-3);
    let x = roughness.ln();
    1.62142 + 0.819955 * x + 0.1734 * x * x + 0.0171201 * x * x * x + 0.000640711 * x * x * x * x
}

fn trowbridge_reitz_sample(
    wi: &Vector3<f64>,
    alpha_x: f64,
    alpha_y: f64,
    u1: f64,
    u2: f64,
) -> Unit<Vector3<f64>> {
    let wi_s = Unit::new_normalize(Vector3::new(alpha_x * wi.x, alpha_y * wi.y, wi.z));
    let slope = trowbridge_reitz_sample_11(bxdf::cos_theta(&wi_s), u1, u2);
    let mut slope_x = slope.x;
    let mut slope_y = slope.y;
    let wi_s_sin_phi = bxdf::sin_phi(&wi_s);
    let wi_s_cos_phi = bxdf::cos_phi(&wi_s);
    // rotate
    let tmp = wi_s_cos_phi * slope_x - wi_s_sin_phi * slope_y;
    slope_y = wi_s_sin_phi * slope_x + wi_s_cos_phi * slope_y;
    slope_x = tmp;
    slope_x = alpha_x * slope_x;
    slope_y = alpha_y * slope_y;
    Unit::new_normalize(Vector3::new(-slope_x, -slope_y, 1.))
}

fn trowbridge_reitz_sample_11(cos_theta: f64, u1: f64, u2: f64) -> Point2<f64> {
    // special case
    if cos_theta > 0.9999 {
        let r = (u1 / (1. - u1)).sqrt();
        let phi = 2. * PI * u2;
        return Point2::new(r * phi.cos(), r * phi.sin());
    }

    let sin_theta = 0f64.max((1. - cos_theta * cos_theta).sqrt());
    let tan_theta = sin_theta / cos_theta;
    let a = 1. / tan_theta;
    let g1 = 2. / (1. + (1. + 1. / (a * a)).sqrt());

    let a = 2. * u1 / g1 - 1.;
    let mut tmp = 1. / (a * a - 1.);
    if tmp > 1e10 {
        // arbitrary cutoff
        tmp = 1e10;
    }
    let b = tan_theta;
    let d = (0f64.max(b * b * tmp * tmp - (a * a - b * b) * tmp)).sqrt();
    let slope_x1 = b * tmp - d;
    let slope_x2 = b * tmp + d;
    let slope_x = if a < 0. || slope_x2 > 1. / tan_theta {
        slope_x1
    } else {
        slope_x2
    };

    let s: f64;
    let nu2: f64;
    if u2 > 0.5 {
        s = 1.;
        nu2 = 2. * (u2 - 0.5);
    } else {
        s = -1.;
        nu2 = 2. * (0.5 - u2);
    }
    let z = (nu2 * (nu2 * (nu2 * 0.27385 - 0.73369) + 0.46341))
        / (nu2 * (nu2 * (nu2 * 0.093073 + 0.309420) - 1.) + 0.597999);
    let slope_y = s * z * (1. + slope_x * slope_x).sqrt();
    Point2::new(slope_x, slope_y)
}
