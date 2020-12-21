pub mod materials {
    use nalgebra::base::Vector3;
    use nalgebra::geometry::{Point3};
    use std::sync::Arc;
    use image::RgbImage;
    use bumpalo::Bump;
    use crate::util;
    use crate::hittable::HitRecord;
    use crate::perlin;
    use crate::microfacet::{trowbridge_reitz_roughness_to_alpha, MicrofacetDistribution};
    use crate::bsdf::Bsdf;
    use crate::bxdf::{Fresnel, Bxdf};
    use crate::geometry::get_objects;
    use crate::consts::*;

    pub enum Material {
        Matte { k_d_id: usize, sigma: f64, bump_id: usize },
        Light { texture_id: usize },
        Plastic { k_d_id: usize, k_s_id: usize, bump_id: usize, roughness: f64, remap_roughness: bool },
        Glass {k_r_id: usize, k_t_id: usize, u_roughness: f64, v_roughness: f64, index: f64, bump_id: usize, remap_roughness: bool },
        Metal {eta_id: usize, k_id: usize, rough_id: usize, urough_id: usize, vrough_id: usize, bump_id: usize, remap_roughness: bool },
        Mirror {color_id: usize, bump_id: usize}
    }
    
    impl Material {
        pub fn compute_scattering_default(record: &mut HitRecord, arena: &Bump) {
            Material::compute_scattering(record, arena, RADIANCE, false)
        }

        pub fn compute_scattering(record: &mut HitRecord, arena: &Bump, mode: u8, allow_lobes: bool) {
            let mat = &get_objects().materials[record.mat_index];
            // TODO: Use arena?
            match mat {
                Material::Matte { k_d_id, sigma, .. } => {
                    // Material::bump(textures, *texture_id, record); // TODO: bump
                    let sig = util::clamp(*sigma, 0., 90.);
                    let color = Texture::value(*k_d_id, record.uv.x, record.uv.y, &record.p);
                    if color != util::black() {
                        record.bsdf = Bsdf::new(record, 1.); // only make bsdf nonempty if we add bxdfs
                        if sig == 0f64 {
                            record.bsdf.add(Bxdf::make_lambertian_reflection(color));
                        } else {
                            record.bsdf.add(Bxdf::make_oren_nayar(color, sig));
                        }
                    }
                }
                Material::Light { .. } => {}
                Material::Plastic { k_d_id, k_s_id, roughness, remap_roughness, .. } => {
                    let color = Texture::value(*k_d_id, record.uv.x, record.uv.y, &record.p);
                    if color != util::black() {
                        record.bsdf = Bsdf::new(record, 1.);
                        record.bsdf.add(Bxdf::make_lambertian_reflection(color));
                    }
                    let specular = Texture::value(*k_s_id, record.uv.x, record.uv.y, &record.p);
                    if specular != util::black() {
                        if Bsdf::is_empty(&record.bsdf) {
                            record.bsdf = Bsdf::new(record, 1.);
                        }
                        let fresnel: Fresnel = Fresnel::FresnelDielectric {eta_i: 1., eta_t: 1.5};
                        let mut rough = *roughness;
                        if *remap_roughness {
                            rough = trowbridge_reitz_roughness_to_alpha(rough);
                        }
                        let distrib = MicrofacetDistribution::make_trowbridge_reitz(rough, rough, true);
                        record.bsdf.add(Bxdf::make_microfacet_reflection(specular, fresnel, distrib));
                    }
                }
                Material::Glass { k_r_id, k_t_id, u_roughness, v_roughness, index, remap_roughness , .. } => {
                    let eta = *index;
                    let mut urough = *u_roughness;
                    let mut vrough = *v_roughness;
                    let r = Texture::value(*k_r_id, record.uv.x, record.uv.y, &record.p);
                    let t = Texture::value(*k_t_id, record.uv.x, record.uv.y, &record.p);
                    record.bsdf = Bsdf::new(record, eta);
                    if r == util::black() && t == util::black() {
                        return;
                    }
                    let is_specular = urough == 0. && vrough == 0.;
                    if is_specular && allow_lobes {
                        record.bsdf.add(Bxdf::make_fresnel_specular(r, t, eta, 1., mode));                    
                    } else {
                        if *remap_roughness {
                            urough = trowbridge_reitz_roughness_to_alpha(urough);
                            vrough = trowbridge_reitz_roughness_to_alpha(vrough);
                        }
                        let mfd = if is_specular {
                            MicrofacetDistribution::Empty
                        } else {
                            MicrofacetDistribution::make_trowbridge_reitz(urough, vrough, true)
                        };
                        if !(r == util::black()) {
                            let fresnel = Fresnel::FresnelDielectric {eta_i: eta, eta_t: 1.};
                            if is_specular {
                                record.bsdf.add(Bxdf::make_specular_reflection(r, fresnel));
                            } else {
                                record.bsdf.add(Bxdf::make_microfacet_reflection(r, fresnel, mfd));
                            }
                        }
                        if !(t == util::black()) {
                            if is_specular {
                                record.bsdf.add(Bxdf::make_specular_transmission(t, eta, 1., mode));
                            } else {
                                record.bsdf.add(Bxdf::make_microfacet_transmission(t, mfd, eta, 1., mode));
                            }
                        }
                    }
                }
                Material::Metal { eta_id, k_id, rough_id, urough_id, vrough_id, remap_roughness, .. } => {
                    // TODO: bump
                    record.bsdf = Bsdf::new(record, 1.);
                    let u_rough = if *urough_id == std::usize::MAX {
                        Texture::value(*rough_id, record.uv.x, record.uv.y, &record.p)
                    } else {
                        Texture::value(*urough_id, record.uv.x, record.uv.y, &record.p)
                    };
                    let v_rough = if *vrough_id == std::usize::MAX {
                        Texture::value(*rough_id, record.uv.x, record.uv.y, &record.p)
                    } else {
                        Texture::value(*vrough_id, record.uv.x, record.uv.y, &record.p)
                    };
                    let uroughness: f64;
                    let vroughness: f64;
                    if *remap_roughness {
                        uroughness = trowbridge_reitz_roughness_to_alpha(u_rough.x);
                        vroughness = trowbridge_reitz_roughness_to_alpha(v_rough.x);
                    } else {
                        uroughness = u_rough.x;
                        vroughness = v_rough.x;
                    }
                    let eta = Texture::value(*eta_id, record.uv.x, record.uv.y, &record.p);
                    let k = Texture::value(*k_id, record.uv.x, record.uv.y, &record.p);
                    let fresnel = Fresnel::FresnelConductor {eta, k};
                    let mfd = MicrofacetDistribution::make_trowbridge_reitz(uroughness, vroughness, true);
                    record.bsdf.add(Bxdf::make_microfacet_reflection(util::white(), fresnel, mfd));
                }
                Material::Mirror { color_id, .. } => {
                    // bump
                    record.bsdf = Bsdf::new(record, 1.);
                    let color = Texture::value(*color_id, record.uv.x, record.uv.y, &record.p);
                    if color != util::black() {
                        record.bsdf.add(Bxdf::make_specular_reflection(color, Fresnel::FresnelNoOp));
                    }
                }
            }
        }
    
        #[allow(unused_variables)]
        pub fn emit(record: &mut HitRecord) -> Vector3<f64> {
            let mat = &get_objects().materials[record.mat_index];
            match mat {
                Material::Light { texture_id } => {
                    Texture::value(*texture_id, record.uv.x, record.uv.y, &record.p)
                }
                _ => {util::black()}
            }
        }
    
        /**
        Sigma should be in the range [0, 90]
        */
        pub fn make_matte(k_d_id: usize, sigma: f64, bump_id: usize) -> Self {
            Material::Matte {k_d_id, sigma, bump_id}
        }
    
        pub fn make_light(texture_id: usize) -> Self {
            Material::Light {texture_id}
        }
    
        pub fn make_plastic(k_d_id: usize, k_s_id: usize, bump_id: usize, roughness: f64, remap_roughness: bool) -> Self {
            Material::Plastic {k_d_id, k_s_id, bump_id, roughness, remap_roughness}
        }
    
        pub fn make_glass(k_r_id: usize, k_t_id: usize, u_roughness: f64, v_roughness: f64, index: f64, bump_id: usize, remap_roughness: bool) -> Self {
            Material::Glass {k_r_id, k_t_id, u_roughness, v_roughness, index, bump_id, remap_roughness }
        }
    
        pub fn make_metal(eta_id: usize, k_id: usize, u_r_id: usize, v_r_id: usize, r_id: usize, bump_id: usize, remap_roughness: bool) -> Self {
            Material::Metal {eta_id, k_id, rough_id: r_id, urough_id: u_r_id, vrough_id: v_r_id, bump_id, remap_roughness }
        }

        pub fn make_mirror(color_id: usize, bump_id: usize) -> Self {
            Material::Mirror {color_id, bump_id}
        }
    }

    #[derive(Clone)]
    pub enum Texture {
        Image {
            img: Arc<RgbImage>,
        },
        SolidColor {
            color: Vector3<f64>,
        },
        Checkered {
            odd: usize,
            even: usize,
            frequency: f64
        },
        Perlin {
            perlin: perlin::Perlin,
        }
    }

    impl Texture {

        pub fn value(index: usize, u: f64, v: f64, p: &Point3<f64>) -> Vector3<f64> {
            match &get_objects().textures[index] {
                Texture::Image { img } => {
                    Texture::image_value(img, u, v, p)
                },
                Texture::SolidColor { color } => { color.clone() },
                Texture::Checkered { even, odd, frequency} => {
                    let mult = (frequency * p.x).sin() * (frequency * p.y).sin() * (frequency * p.z).sin();
                    if mult < 0. {
                        Texture::value(*even, u, v, p)
                    } else {
                        Texture::value(*odd, u, v, p)
                    }
                }
                Texture::Perlin { perlin } => {
                    let noise = perlin.turb(p, 7);
                    util::white().scale(0.5 * (1. + (perlin.scale * p.z + 10. * noise).sin()))
                }
            }
        }

        fn image_value(img: &Arc<RgbImage>, u: f64, v: f64, _p: &Point3<f64>) -> Vector3<f64> {
            let mut x: u32 = (u * img.width() as f64).round() as u32;
            let mut y: u32 = ((1. - v) * img.height() as f64).round() as u32;
            if y == img.height() {
                y = y - 1;
            }
            if x == img.width() {
                x = x - 1;
            }
            let from_image = img.get_pixel(x, y);
            let r: f64 = from_image[0] as f64 / 255.;
            let g: f64 = from_image[1] as f64 / 255.;
            let b: f64 = from_image[2] as f64 / 255.;
            Vector3::new(r, g, b)
        }

        pub fn new_texture(path: &str) -> Self {
            let img: RgbImage = image::open(path).unwrap().to_rgb8();
            Texture::Image { img: Arc::new(img) }
        }

        pub fn new_solid_color(color: Vector3<f64>) -> Self { Texture::SolidColor { color } }
        pub fn new_checkered(even: usize, odd: usize, frequency: f64) -> Self {
            Texture::Checkered { even, odd, frequency }
        }
        pub fn new_perlin(scale: f64) -> Self {
            let perlin = perlin::Perlin::new(256, scale);
            Texture::Perlin { perlin }
        }
    }
}