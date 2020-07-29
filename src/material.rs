pub mod materials {
    use nalgebra::base::{Unit, Vector3};
    use nalgebra::geometry::{Point3};
    use std::sync::Arc;
    use crate::util::*;
    use crate::hittable::HitRecord;
    use crate::geometry::Ray;
    use crate::perlin;
    use image::RgbImage;

    #[derive(Clone)]
    pub enum Material {
        Lambertian {
            texture_index: usize,
        },
        Metal {
            albedo: Vector3<f64>,
            roughness: f64,
        },
        Dielectric {
            index: f64,
        },
        DiffuseLight {
            texture_index: usize,
        },
        Isotropic {
            texture_index: usize,
        }
    }

    impl Material {
        pub fn scatter(materials: &Vec<Material>, index: usize, textures: &Vec<Texture>, ray: &Ray, hit_record: &HitRecord) -> Option<(Ray, Vector3<f64>)> {
            match &materials[index] {
                Material::Lambertian { texture_index } => {
                    let dir: Vector3<f64> = hit_record.n.as_ref() + rand_in_unit_sphere();
                    let ray = Ray::new_time(hit_record.p, dir, ray.time);
                    let color = Texture::value(textures, *texture_index, hit_record.uv.x, hit_record.uv.y, &hit_record.p);
                    Some((ray, color))
                }
                Material::Metal { albedo, roughness } => {
                    let refl: Vector3<f64> = reflect(&ray.dir, &hit_record.n);
                    let new_ray = Ray::new_time(hit_record.p, refl + rand_in_unit_sphere().scale(*roughness), ray.time);
                    if refl.dot(hit_record.n.as_ref()) <= 0. {
                        None
                    } else {
                        Some((new_ray, *albedo))
                    }
                }
                Material::Dielectric { index } => {
                    let etai: f64 = if hit_record.front { 1. / index } else { *index };
                    let unit_dir: Unit<Vector3<f64>> = Unit::new_normalize(ray.dir);
                    let cost = hit_record.n.dot(&-unit_dir).min(1.);
                    let sint = (1. - cost * cost).sqrt();
                    let prob = schlick(cost, etai);
                    let new_dir: Vector3<f64> = if etai * sint > 1. || rand() < prob { reflect(unit_dir.as_ref(), &hit_record.n) } else { refract(&unit_dir, &hit_record.n, etai) };
                    return Some((Ray::new_time(hit_record.p, new_dir, ray.time), Vector3::new(255., 255., 255.)));
                }
                Material::DiffuseLight { texture_index: _ } => { None }
                Material::Isotropic { texture_index } => {
                    let new_ray = Ray::new_time(hit_record.p, rand_in_unit_sphere(), ray.time);
                    let color = Texture::value(textures, *texture_index, hit_record.uv.x, hit_record.uv.y, &hit_record.p);
                    Some((new_ray, color))
                }
            }
        }

        #[allow(unused_variables)]
        pub fn emit(materials: &Vec<Material>, index: usize, textures: &Vec<Texture>, u: f64, v: f64, p: &Point3<f64>) -> Vector3<f64> {
            match &materials[index] {
                Material::Lambertian { texture_index } => {Vector3::new(0., 0., 0.)}
                Material::Metal { albedo, roughness } => {Vector3::new(0., 0., 0.)}
                Material::Dielectric { index } => {Vector3::new(0., 0., 0.)}
                Material::DiffuseLight { texture_index } => { Texture::value(textures, *texture_index, u, v, p) }
                Material::Isotropic { texture_index } => {Vector3::new(0., 0., 0.)}
            }
        }
        pub fn new_lambertian(texture_index: usize) -> Self { Material::Lambertian { texture_index } }
        pub fn new_metal(albedo: Vector3<f64>, roughness: f64) -> Self { Material::Metal { albedo, roughness } }
        pub fn new_dielectric(index: f64) -> Self { Material::Dielectric { index } }
        pub fn new_diffuse(texture_index: usize) -> Self { Material::DiffuseLight { texture_index } }
        pub fn new_isotropic(texture_index: usize) -> Self { Material::Isotropic { texture_index } }

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
            odd: Box<Texture>,
            even: Box<Texture>,
            frequency: f64
        },
        Perlin {
            perlin: perlin::Perlin,
        }
    }

    impl Texture {

        pub fn value(textures: &Vec<Texture>, index: usize, u: f64, v: f64, p: &Point3<f64>) -> Vector3<f64> {
            match &textures[index] {
                Texture::Image { img } => {
                    Texture::image_value(img, u, v, p)
                },
                Texture::SolidColor { color } => { color.clone() },
                Texture::Checkered { even, odd, frequency} => {
                    let mult = (frequency * p.x).sin() * (frequency * p.y).sin() * (frequency * p.z).sin();
                    if mult < 0. {
                        Texture::value_of_obj(even.as_ref(), u, v, p)
                    } else {
                        Texture::value_of_obj(odd.as_ref(), u, v, p)
                    }
                }
                Texture::Perlin { perlin } => {
                    let noise = perlin.turb(p, 7);
                    // Vector3::new(255., 255., 255.).scale(noise) //0.5 * (1. + noise))
                    Vector3::new(255., 255., 255.).scale(0.5 * (1. + (perlin.scale * p.z + 10. * noise).sin()))
                }
            }
        }

        pub fn value_of_obj(texture: &Texture, u: f64, v: f64, p: &Point3<f64>) -> Vector3<f64> {
            match &texture {
                Texture::Image { img } => {
                    Texture::image_value(img, u, v, p)
                },
                Texture::SolidColor { color } => { color.clone() },
                Texture::Checkered { even, odd, frequency} => {
                    let mult = (frequency * p.x).sin() * (frequency * p.y).sin() * (frequency * p.z).sin();
                    if mult < 0. {
                        Texture::value_of_obj(even.as_ref(), u, v, p)
                    } else {
                        Texture::value_of_obj(odd.as_ref(), u, v, p)
                    }
                }
                Texture::Perlin { perlin } => {
                    let noise = perlin.turb(p, 7);
                    // Vector3::new(255., 255., 255.).scale(noise) //0.5 * (1. + noise))
                    Vector3::new(255., 255., 255.).scale(0.5 * (1. + (perlin.scale * p.z + 10. * noise).sin()))
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
            let r: f64 = from_image[0] as f64;
            let g: f64 = from_image[1] as f64;
            let b: f64 = from_image[2] as f64;
            Vector3::new(r, g, b)
        }

        pub fn new_texture(path: &str) -> Self {
            let img: RgbImage = image::open(path).unwrap().to_rgb();
            Texture::Image { img: Arc::new(img) }
        }

        pub fn new_solid_color(color: Vector3<f64>) -> Self { Texture::SolidColor { color } }
        pub fn new_checkered(even: Box<Texture>, odd: Box<Texture>, frequency: f64) -> Self {
            Texture::Checkered { even, odd, frequency }
        }
        pub fn new_perlin(scale: f64) -> Self {
            let perlin = perlin::Perlin::new(256, scale);
            Texture::Perlin { perlin }
        }
    }
}