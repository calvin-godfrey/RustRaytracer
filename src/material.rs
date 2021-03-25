pub mod materials {
    use crate::bsdf::Bsdf;
    use crate::bxdf::{eta_to_r0, Bxdf, Fresnel};
    use crate::consts::*;
    use crate::geometry::get_objects;
    use crate::hittable::HitRecord;
    use crate::microfacet::{trowbridge_reitz_roughness_to_alpha, MicrofacetDistribution};
    use crate::perlin;
    use crate::util;
    use bumpalo::Bump;
    use image::{Rgb, RgbImage};
    use nalgebra::base::Vector3;
    use nalgebra::geometry::{Point2, Point3};
    use std::fs::File;
    use std::{io::BufReader, sync::Arc};

    pub enum Material {
        Matte {
            k_d_id: usize,
            sigma: f64,
            bump_id: usize,
        },
        Light {
            texture_id: usize,
        },
        Plastic {
            k_d_id: usize,
            k_s_id: usize,
            bump_id: usize,
            roughness: f64,
            remap_roughness: bool,
        },
        Glass {
            k_r_id: usize,
            k_t_id: usize,
            u_roughness: f64,
            v_roughness: f64,
            index: f64,
            bump_id: usize,
            remap_roughness: bool,
        },
        Metal {
            eta_id: usize,
            k_id: usize,
            rough_id: usize,
            urough_id: usize,
            vrough_id: usize,
            bump_id: usize,
            remap_roughness: bool,
        },
        Mirror {
            color_id: usize,
            bump_id: usize,
        },
        Disney {
            c_id: usize,
            metallic_id: usize,
            eta_id: usize,
            roughness_id: usize,
            spec_tint_id: usize,
            anisotropic_id: usize,
            sheen_id: usize,
            sheen_tint_id: usize,
            clearcoat_id: usize,
            clearcoat_gloss_id: usize,
            spec_trans_id: usize,
            scatter_distance_id: usize,
            thin: bool,
            flatness_id: usize,
            diff_trans_id: usize,
            bump_id: usize,
        },
    }

    impl Material {
        pub fn compute_scattering_default(record: &mut HitRecord, arena: &Bump) {
            Material::compute_scattering(record, arena, RADIANCE, false)
        }

        pub fn compute_scattering(
            record: &mut HitRecord,
            arena: &Bump,
            mode: u8,
            allow_lobes: bool,
        ) {
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
                Material::Plastic {
                    k_d_id,
                    k_s_id,
                    roughness,
                    remap_roughness,
                    ..
                } => {
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
                        let fresnel: Fresnel = Fresnel::FresnelDielectric {
                            eta_i: 1.5,
                            eta_t: 1.,
                        };
                        let mut rough = *roughness;
                        if *remap_roughness {
                            rough = trowbridge_reitz_roughness_to_alpha(rough);
                        }
                        let distrib =
                            MicrofacetDistribution::make_trowbridge_reitz(rough, rough, true);
                        record
                            .bsdf
                            .add(Bxdf::make_microfacet_reflection(specular, fresnel, distrib));
                    }
                }
                Material::Glass {
                    k_r_id,
                    k_t_id,
                    u_roughness,
                    v_roughness,
                    index,
                    remap_roughness,
                    ..
                } => {
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
                        record
                            .bsdf
                            .add(Bxdf::make_fresnel_specular(r, t, eta, 1., mode));
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
                            let fresnel = Fresnel::FresnelDielectric {
                                eta_i: eta,
                                eta_t: 1.,
                            };
                            if is_specular {
                                record.bsdf.add(Bxdf::make_specular_reflection(r, fresnel));
                            } else {
                                record
                                    .bsdf
                                    .add(Bxdf::make_microfacet_reflection(r, fresnel, mfd));
                            }
                        }
                        if !(t == util::black()) {
                            if is_specular {
                                record
                                    .bsdf
                                    .add(Bxdf::make_specular_transmission(t, eta, 1., mode));
                            } else {
                                record
                                    .bsdf
                                    .add(Bxdf::make_microfacet_transmission(t, mfd, eta, 1., mode));
                            }
                        }
                    }
                }
                Material::Metal {
                    eta_id,
                    k_id,
                    rough_id,
                    urough_id,
                    vrough_id,
                    remap_roughness,
                    ..
                } => {
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
                    let fresnel = Fresnel::FresnelConductor { eta, k };
                    let mfd =
                        MicrofacetDistribution::make_trowbridge_reitz(uroughness, vroughness, true);
                    record.bsdf.add(Bxdf::make_microfacet_reflection(
                        util::white(),
                        fresnel,
                        mfd,
                    ));
                }
                Material::Mirror { color_id, .. } => {
                    // bump
                    record.bsdf = Bsdf::new(record, 1.);
                    let color = Texture::value(*color_id, record.uv.x, record.uv.y, &record.p);
                    if color != util::black() {
                        record
                            .bsdf
                            .add(Bxdf::make_specular_reflection(color, Fresnel::FresnelNoOp));
                    }
                }
                Material::Disney {
                    c_id,
                    metallic_id,
                    eta_id,
                    roughness_id,
                    spec_tint_id,
                    anisotropic_id,
                    sheen_id,
                    sheen_tint_id,
                    clearcoat_id,
                    clearcoat_gloss_id,
                    spec_trans_id,
                    scatter_distance_id,
                    thin,
                    flatness_id,
                    diff_trans_id,
                    bump_id,
                } => {
                    let u = record.uv.x;
                    let v = record.uv.y;
                    let p = &record.p;
                    let color = Texture::value(*c_id, u, v, p);
                    let metallic_weight = Texture::value(*metallic_id, u, v, p).x;
                    let e = Texture::value(*eta_id, u, v, p).x;
                    let strans = Texture::value(*spec_trans_id, u, v, p).x;
                    let diffuse_weight = (1f64 - metallic_weight) * (1f64 - strans);
                    // 0 means all diffuse is reflected, 1 means all is transmitted
                    let dt = Texture::value(*diff_trans_id, u, v, p).x / 2f64;
                    let rough = Texture::value(*roughness_id, u, v, p).x;
                    let lum = util::color_to_luminance(&color);
                    // normalize color
                    let c_tint: Vector3<f64> = if lum > 0f64 {
                        1f64 / lum * color
                    } else {
                        util::white()
                    };
                    let sheen_weight = Texture::value(*sheen_id, u, v, p).x;
                    let mut c_sheen = util::black();
                    if sheen_weight > 0f64 {
                        let sheen_tint = Texture::value(*sheen_tint_id, u, v, p).x;
                        c_sheen = util::lerp_v(sheen_tint, &util::white(), &c_tint);
                    }

                    if diffuse_weight > 0f64 {
                        if *thin {
                            let flat = Texture::value(*flatness_id, u, v, p).x;
                            // blend between diffuse and fake susurface; weight using dt
                            record.bsdf.add(Bxdf::make_disney_diffuse(
                                diffuse_weight * (1f64 - flat) * (1f64 - dt) * color,
                            ));
                            record.bsdf.add(Bxdf::make_disney_fake_ss(
                                diffuse_weight * flat * (1f64 - dt) * color,
                                rough,
                            ));
                        } else {
                            let sd = Texture::value(*scatter_distance_id, u, v, p);
                            if sd == util::black() {
                                // no subsurface scattering
                                record
                                    .bsdf
                                    .add(Bxdf::make_disney_diffuse(diffuse_weight * color));
                            } else {
                                // use bssrdf instead; right now, that doesn't exist, so just specular transmission (?) TODO:
                                record.bsdf.add(Bxdf::make_specular_transmission(
                                    util::white(),
                                    1f64,
                                    e,
                                    mode,
                                ))
                            }
                        }
                        // add retro-reflection
                        record
                            .bsdf
                            .add(Bxdf::make_disney_retro(diffuse_weight * color, rough));

                        // and sheen (if necessary)
                        if sheen_weight > 0f64 {
                            record.bsdf.add(Bxdf::make_disney_sheen(
                                diffuse_weight * sheen_weight * c_sheen,
                            ));
                        }
                    }

                    // now do microfacet transmission/reflection
                    let ani = Texture::value(*anisotropic_id, u, v, p).x;
                    let aspect = (1f64 - ani * 0.9).sqrt(); // TODO: why 0.9?
                    let ax = 0.001f64.max(rough * rough / aspect);
                    let ay = 0.001f64.max(rough * rough * aspect);
                    let dist = MicrofacetDistribution::make_disney(ax, ay);
                    let spec_tint = Texture::value(*spec_tint_id, u, v, p).x;
                    let c_spec_0 = util::lerp_v(
                        metallic_weight,
                        &(eta_to_r0(e) * util::lerp_v(spec_tint, &util::white(), &c_tint)),
                        &color,
                    );
                    let fresnel = Fresnel::DisneyFresnel {
                        r0: c_spec_0,
                        metallic: metallic_weight,
                        eta: e,
                    };
                    record.bsdf.add(Bxdf::make_microfacet_reflection(
                        util::white(),
                        fresnel,
                        dist,
                    ));

                    // clearcoat
                    let cc = Texture::value(*clearcoat_id, u, v, p).x;
                    if cc > 0f64 {
                        let gloss = Texture::value(*clearcoat_gloss_id, u, v, p).x;
                        record.bsdf.add(Bxdf::make_disney_clearcoat(
                            cc,
                            util::lerp(gloss, 0.1, 0.001),
                        ));
                    }

                    // transmission
                    if strans > 0f64 {
                        let t = Vector3::new(
                            strans * color.x.sqrt(),
                            strans * color.y.sqrt(),
                            strans * color.z.sqrt(),
                        );
                        if *thin {
                            // approximate with scaled distribution
                            let rscaled = (0.65 * e - 0.35) * rough; // from Burley 2015
                            let ax = 0.001f64.max(rscaled * rscaled / aspect);
                            let ay = 0.001f64.max(rscaled * rscaled * aspect);
                            let scaled_dist =
                                MicrofacetDistribution::make_trowbridge_reitz(ax, ay, true);
                            record.bsdf.add(Bxdf::make_microfacet_transmission(
                                t,
                                scaled_dist,
                                1f64,
                                e,
                                mode,
                            ));
                        } else {
                            record
                                .bsdf
                                .add(Bxdf::make_microfacet_transmission(t, dist, 1f64, e, mode));
                        }
                    }
                    if *thin {
                        record
                            .bsdf
                            .add(Bxdf::make_lambertian_transmission(dt * color));
                    }
                }
            }
        }

        /**
        Sigma should be in the range [0, 90]
        */
        pub fn make_matte(k_d_id: usize, sigma: f64, bump_id: usize) -> Self {
            Material::Matte {
                k_d_id,
                sigma,
                bump_id,
            }
        }

        pub fn make_light(texture_id: usize) -> Self {
            Material::Light { texture_id }
        }

        pub fn make_plastic(
            k_d_id: usize,
            k_s_id: usize,
            bump_id: usize,
            roughness: f64,
            remap_roughness: bool,
        ) -> Self {
            Material::Plastic {
                k_d_id,
                k_s_id,
                bump_id,
                roughness,
                remap_roughness,
            }
        }

        pub fn make_glass(
            k_r_id: usize,
            k_t_id: usize,
            u_roughness: f64,
            v_roughness: f64,
            index: f64,
            bump_id: usize,
            remap_roughness: bool,
        ) -> Self {
            Material::Glass {
                k_r_id,
                k_t_id,
                u_roughness,
                v_roughness,
                index,
                bump_id,
                remap_roughness,
            }
        }

        pub fn make_metal(
            eta_id: usize,
            k_id: usize,
            u_r_id: usize,
            v_r_id: usize,
            r_id: usize,
            bump_id: usize,
            remap_roughness: bool,
        ) -> Self {
            Material::Metal {
                eta_id,
                k_id,
                rough_id: r_id,
                urough_id: u_r_id,
                vrough_id: v_r_id,
                bump_id,
                remap_roughness,
            }
        }

        pub fn make_mirror(color_id: usize, bump_id: usize) -> Self {
            Material::Mirror { color_id, bump_id }
        }

        /**
        All of the parameters should have the texture value in the x coordinate
        except for c_id and scatter_distance_id
        */
        pub fn make_disney(
            c_id: usize,
            metallic_id: usize,
            eta_id: usize,
            roughness_id: usize,
            spec_tint_id: usize,
            anisotropic_id: usize,
            sheen_id: usize,
            sheen_tint_id: usize,
            clearcoat_id: usize,
            clearcoat_gloss_id: usize,
            spec_trans_id: usize,
            scatter_distance_id: usize,
            thin: bool,
            flatness_id: usize,
            diff_trans_id: usize,
            bump_id: usize,
        ) -> Self {
            Material::Disney {
                c_id,
                metallic_id,
                eta_id,
                roughness_id,
                spec_tint_id,
                anisotropic_id,
                sheen_id,
                sheen_tint_id,
                clearcoat_id,
                clearcoat_gloss_id,
                spec_trans_id,
                scatter_distance_id,
                thin,
                flatness_id,
                diff_trans_id,
                bump_id,
            }
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
            frequency: f64,
        },
        Perlin {
            perlin: perlin::Perlin,
        },
        Hdr {
            meta: image::hdr::HdrMetadata,
            data: Vec<Rgb<f32>>,
        },
    }

    impl Texture {
        pub fn value(index: usize, u: f64, v: f64, p: &Point3<f64>) -> Vector3<f64> {
            Texture::get_value(&get_objects().textures[index], u, v, p)
        }

        /**
        p is only used to checkered and perlin
        */
        pub fn get_value(&self, u: f64, v: f64, p: &Point3<f64>) -> Vector3<f64> {
            match self {
                Texture::Image { img } => Texture::image_value(img, u, v),
                Texture::SolidColor { color } => color.clone(),
                Texture::Checkered {
                    even,
                    odd,
                    frequency,
                } => {
                    let mult =
                        (frequency * u * 2f64 * PI).sin() * (frequency * v * 2f64 * PI).sin();
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
                Texture::Hdr { meta, data } => {
                    let width = meta.width;
                    let height = meta.height;
                    let x = ((1f64 - u) * width as f64).round() as u32;
                    let y = (v * height as f64).round() as u32;
                    let x = x % width;
                    let y = y % height;
                    let rgb = data[(y * width + x) as usize];
                    let rgbe = image::hdr::to_rgbe8(rgb);
                    let r = rgbe.c[0] as u32;
                    let g = rgbe.c[1] as u32;
                    let b = rgbe.c[2] as u32;
                    let v = (2f64).powi(rgbe.e as i32 - 128);
                    let r = (r as f64 + 0.5) * v / 256f64;
                    let g = (g as f64 + 0.5) * v / 256f64;
                    let b = (b as f64 + 0.5) * v / 256f64;
                    Vector3::new(r, g, b)
                }
            }
        }

        pub fn get_size(&self) -> Point2<u32> {
            match self {
                Texture::Image { img } => Point2::new(img.width(), img.height()),
                Texture::SolidColor { .. } => Point2::new(1, 1),
                Texture::Checkered { .. } => Point2::origin(),
                Texture::Perlin { .. } => Point2::origin(),
                Texture::Hdr { meta, .. } => Point2::new(meta.width, meta.height),
            }
        }

        fn image_value(img: &Arc<RgbImage>, u: f64, v: f64) -> Vector3<f64> {
            let mut x: u32 = (u * img.width() as f64).round() as u32;
            let mut y: u32 = ((1. - v) * img.height() as f64).round() as u32;
            x = x % img.width(); // wrap image in x
            y = y % img.height(); // and y directions
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

        pub fn new_solid_color(color: Vector3<f64>) -> Self {
            Texture::SolidColor { color }
        }
        pub fn new_checkered(even: usize, odd: usize, frequency: f64) -> Self {
            Texture::Checkered {
                even,
                odd,
                frequency,
            }
        }
        pub fn new_perlin(scale: f64) -> Self {
            let perlin = perlin::Perlin::new(256, scale);
            Texture::Perlin { perlin }
        }
        pub fn new_hdr(path: &str) -> Self {
            let file = File::open(path).expect(&format!("Unable to open file {}", path));
            let f = BufReader::new(file);
            let decoder =
                image::hdr::HdrDecoder::new(f).expect(&format!("Failure decoding hdr {}", path));
            let meta = decoder.metadata();
            let data = decoder
                .read_image_hdr()
                .expect(&format!("Failure to decode data from hdr {}", path));
            Texture::Hdr { meta, data }
        }
    }
}
