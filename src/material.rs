pub mod materials {
    use nalgebra::base::{Unit, Vector3};
    use crate::util::*;
    use std::sync::Arc;
    use crate::hittable::HitRecord;
    use crate::geometry::Ray;
    use image::RgbImage;

    #[derive(Clone)]
    pub enum Material {
        Lambertian {
            albedo: Vector3<f64>,
        },
        Metal {
            albedo: Vector3<f64>,
            roughness: f64,
        },
        Dielectric {
            index: f64,
        },
        Texture {
            img: Arc<RgbImage>,
            albedo: Vector3<f64>,
        }
    }

    impl Material {
        pub fn scatter(ray: &Ray, hit_record: &HitRecord) -> Option<(Ray, Vector3<f64>)> {
            match (*hit_record.mat).clone() {
                Material::Lambertian { albedo } => {
                    let dir: Vector3<f64> = hit_record.n.as_ref() + rand_in_hemisphere(hit_record.n.as_ref());
                    let ray = Ray::new_time(hit_record.p, dir, ray.time);
                    let color = albedo;
                    Some((ray, color))
                }
                Material::Metal { albedo, roughness } => {
                    let refl: Vector3<f64> = reflect(&ray.dir, &hit_record.n);
                    let new_ray = Ray::new_time(hit_record.p, refl + rand_in_unit_sphere().scale(roughness), ray.time);
                    if refl.dot(hit_record.n.as_ref()) <= 0. {
                        None
                    } else {
                        Some((new_ray, albedo))
                    }
                }
                Material::Dielectric { index } => {
                    let etai: f64 = if hit_record.front { 1. / index } else { index };
                    let unit_dir: Unit<Vector3<f64>> = Unit::new_normalize(ray.dir);
                    let cost = hit_record.n.dot(&-unit_dir).min(1.);
                    let sint = (1. - cost * cost).sqrt();
                    let prob = schlick(cost, etai);
                    let new_dir: Vector3<f64> = if etai * sint > 1. || rand() < prob { reflect(unit_dir.as_ref(), &hit_record.n) } else { refract(&unit_dir, &hit_record.n, etai) };
                    return Some((Ray::new_time(hit_record.p, new_dir, ray.time), Vector3::new(255., 255., 255.)));
                }
                Material::Texture { img, albedo } => {
                    let dir: Vector3<f64> = hit_record.n.as_ref() + rand_in_hemisphere(hit_record.n.as_ref());
                    let ray = Ray::new_time(hit_record.p, dir, ray.time);
                    let color: Vector3<f64>;

                    match hit_record.uv {
                        Some(coords) => {
                            let u: u32 = (coords.x * img.width() as f64).round() as u32;
                            let v: u32 = ((1. - coords.y) * img.height() as f64).round() as u32;
                            let from_image = img.get_pixel(u, v);
                            let r: f64 = from_image[0] as f64;
                            let g: f64 = from_image[1] as f64;
                            let b: f64 = from_image[2] as f64;
                            color = Vector3::new(r, g, b);
                        }, // get color from texture
                        None => color = albedo,
                    }
                    Some((ray, color))
                }
            }
        }
        pub fn new_lambertian(albedo: Vector3<f64>) -> Self { Material::Lambertian { albedo } }
        pub fn new_metal(albedo: Vector3<f64>, roughness: f64) -> Self { Material::Metal { albedo, roughness } }
        pub fn new_dielectric(index: f64) -> Self { Material::Dielectric { index } }
        pub fn new_texture(path: &str, albedo: Vector3<f64>) -> Self {
            let img: RgbImage = image::open(path).unwrap().to_rgb();
            Material::Texture { img: Arc::new(img), albedo }
        }
    }
}