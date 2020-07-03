pub mod materials {
    use nalgebra::base::Vector3;
    use crate::util::{rand_in_hemisphere, rand_in_unit_sphere, reflect, refract};
    use crate::hittable::HitRecord;
    use crate::geometry::Ray;

    pub trait Material {
        fn scatter(&self, ray: &Ray, hit_record: &HitRecord) -> (Option<Ray>, Option<Vector3<f64>>);
    }
    
    pub struct Lambertian {
        albedo: Vector3<f64>,
    }
    
    impl Material for Lambertian {
        fn scatter(&self, ray: &Ray, hit_record: &HitRecord) -> (Option<Ray>, Option<Vector3<f64>>) {
            let dir: Vector3<f64> = hit_record.n.as_ref() + rand_in_hemisphere(hit_record.n.as_ref());
            let ray = Ray::new(hit_record.p, dir);
            let color = self.albedo;
            (Some(ray), Some(color))
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
        fn scatter(&self, ray: &Ray, hit_record: &HitRecord) -> (Option<Ray>, Option<Vector3<f64>>) {
            let refl: Vector3<f64> = reflect(&ray.dir, &hit_record.n);
            let new_ray = Ray::new(hit_record.p, refl + rand_in_unit_sphere().scale(self.roughness));
            if refl.dot(hit_record.n.as_ref()) <= 0. {
                (None, None)
            } else {
                (Some(new_ray), Some(self.albedo))
            }
        }
    }
    
    impl Metal {
        pub fn new(albedo: Vector3<f64>, roughness: f64) -> Self { Self { albedo, roughness } }
    }
}