pub mod materials {
    use nalgebra::base::{Unit, Vector3};
    use crate::util::{rand_in_hemisphere, rand_in_unit_sphere, reflect, refract, schlick, rand};
    use crate::hittable::HitRecord;
    use crate::geometry::Ray;
    use image::RgbImage;

    pub trait Material {
        fn scatter(&self, ray: &Ray, hit_record: &HitRecord) -> Option<(Ray, Vector3<f64>)>;
    }
    
    pub struct Lambertian {
        albedo: Vector3<f64>,
    }
    
    impl Material for Lambertian {
        #[allow(unused_variables)]
        fn scatter(&self, ray: &Ray, hit_record: &HitRecord) -> Option<(Ray, Vector3<f64>)> {
            let dir: Vector3<f64> = hit_record.n.as_ref() + rand_in_hemisphere(hit_record.n.as_ref());
            let ray = Ray::new(hit_record.p, dir);
            let color = self.albedo;
            Some((ray, color))
        }
    }
    
    impl Lambertian {
        pub fn new(albedo: Vector3<f64>) -> Self { Self { albedo } }
    }
    
    pub struct Metal {
        albedo: Vector3<f64>,
        roughness: f64,
    }
    
    impl Material for Metal {
        fn scatter(&self, ray: &Ray, hit_record: &HitRecord) -> Option<(Ray, Vector3<f64>)> {
            let refl: Vector3<f64> = reflect(&ray.dir, &hit_record.n);
            let new_ray = Ray::new(hit_record.p, refl + rand_in_unit_sphere().scale(self.roughness));
            if refl.dot(hit_record.n.as_ref()) <= 0. {
                None
            } else {
                Some((new_ray, self.albedo))
            }
        }
    }
    
    impl Metal {
        pub fn new(albedo: Vector3<f64>, roughness: f64) -> Self { Self { albedo, roughness } }
    }

    pub struct Dielectric {
        index: f64,
    }

    impl Material for Dielectric {
        fn scatter(&self, ray: &Ray, hit_record: &HitRecord) -> Option<(Ray, Vector3<f64>)> {
            let etai: f64 = if hit_record.front { 1. / self.index } else { self.index };
            let unit_dir: Unit<Vector3<f64>> = Unit::new_normalize(ray.dir);
            let cost = hit_record.n.dot(&-unit_dir).min(1.);
            let sint = (1. - cost * cost).sqrt();
            let prob = schlick(cost, etai);
            let new_dir: Vector3<f64> = if etai * sint > 1. || rand() < prob { reflect(unit_dir.as_ref(), &hit_record.n) } else { refract(&unit_dir, &hit_record.n, etai) };
            return Some((Ray::new(hit_record.p, new_dir), Vector3::new(255., 255., 255.)));
        }
    }

    impl Dielectric {
        pub fn new(index: f64) -> Self { Self { index } }
    }

    pub struct Texture {
        img: RgbImage,
        backup: Vector3<f64>,
    }

    impl Material for Texture {
        #[allow(unused_variables)]
        fn scatter(&self, ray: &Ray, hit_record: &HitRecord) -> Option<(Ray, Vector3<f64>)> {
            let dir: Vector3<f64> = hit_record.n.as_ref() + rand_in_hemisphere(hit_record.n.as_ref());
            let ray = Ray::new(hit_record.p, dir);
            let color: Vector3<f64>;

            match hit_record.uv {
                Some(coords) => {
                    let u: u32 = (coords.x * self.img.width() as f64).round() as u32;
                    let v: u32 = ((1. - coords.y) * self.img.height() as f64).round() as u32;
                    let from_image = self.img.get_pixel(u, v);
                    let r: f64 = from_image[0] as f64;
                    let g: f64 = from_image[1] as f64;
                    let b: f64 = from_image[2] as f64;
                    // if r == 0. && g == 0. && b == 0. {
                    //     println!("Bad?");
                    // }
                    color = Vector3::new(r, g, b);
                }, // get color from texture
                None => color = self.backup,
            }

            Some((ray, color))
        }
    }

    impl Texture {
        pub fn new(path: &str, backup: Vector3<f64>) -> Self {
            let img: RgbImage = image::open(path).unwrap().to_rgb();
            Self { img, backup }
        }
    }
}