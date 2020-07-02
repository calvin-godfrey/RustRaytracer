extern crate image;


use image::{Rgb, RgbImage};
use nalgebra::base::{Unit, Vector3};
use nalgebra::geometry::Point3;

trait Hittable {
    fn intersects(&self, ray: &Ray, tmin: f64, tmax: f64) -> Option<HitRecord>;
}

struct HitRecord {
    t: f64, // time of hit along ray
    n: Unit<Vector3<f64>>, // normal of surface at point
    p: Point3<f64>, // point of intersection
}

impl HitRecord {
    fn new(t: f64, n: Unit<Vector3<f64>>, p: Point3<f64>) -> Self { Self {t, n, p } }
}

struct Ray {
    origin: Point3<f64>,
    dir: Vector3<f64>,
}

impl Ray {
    fn new(origin: Point3<f64>, dir: Vector3<f64>) -> Self { Self { origin, dir } }

    fn at(&self, t: f64) -> Point3<f64> {
        self.origin + self.dir.scale(t)
    }
}

struct Sphere {
    center: Point3<f64>,
    r: f64,
}

impl Sphere {
    fn new(center: Point3<f64>, r: f64) -> Self { Self { center, r } }
}

impl Hittable for Sphere {
    
    fn intersects(&self, ray: &Ray, tmin: f64, tmax: f64) -> Option<HitRecord> {
        let diff: Vector3<f64> = ray.origin - self.center;
        // get quadratic equation, calculate discriminant
        let a = ray.dir.dot(&ray.dir);
        let b = diff.dot(&ray.dir);
        let c = diff.dot(&diff) - self.r * self.r;
        let disc = b * b - a * c;
        if disc < 0.0 {
            return None;
        }
        let inv_a = 1.0 / a;
        let root = disc.sqrt();
        let ans = (-b - root) * inv_a; // try first solution to equation
        if ans < tmax && ans > tmin {
            let hit = ray.at(ans);
            let hit_record = HitRecord::new(ans, Unit::new_normalize(self.center - hit), hit);
            return Some(hit_record);
        }
        let ans = (-b + root) * inv_a;
        if ans < tmax && ans > tmin {
            let hit = ray.at(ans);
            let hit_record = HitRecord::new(ans, Unit::new_normalize(self.center - hit), hit);
            return Some(hit_record);
        } else {
            return None;
        }
    }
}


fn main() {
    let image_width: u32 = 512;
    let aspect_ratio = 3.0 / 2.0;
    let image_height: u32 = ((image_width as f64) / aspect_ratio).round() as u32;
    let viewport_height = 2.0;
    let viewport_width = viewport_height * aspect_ratio;
    let focal_length = 1.0;

    let origin: Point3<f64> = Point3::origin();
    let horizontal_offset: Vector3<f64> = Vector3::new(viewport_width, 0.0, 0.0);
    let vertical_offset: Vector3<f64> = Vector3::new(0.0, viewport_height, 0.0);
    // this is the point in world space that represents the bottom left corner of the plane that is being projected onto
    let bottom_left_corner: Point3<f64> = origin - horizontal_offset.scale(0.5) - vertical_offset.scale(0.5) - Vector3::new(0.0, 0.0, focal_length);
    let mut img = RgbImage::new(image_width, image_height);

    let sphere: Sphere = Sphere::new(Point3::new(0.0, 0.0, -1.0), 0.5);

    let sphere_array = [sphere];
    let light: Point3<f64> = Point3::new(0.0, 0.0, 0.0);

    for i in 0u32..image_width {
        for j in 0u32..image_height {
            let u: f64 = (i as f64) / ((image_width - 1) as f64);
            let v: f64 = (j as f64) / ((image_height - 1) as f64);
            let to: Point3<f64> = bottom_left_corner + horizontal_offset.scale(u) + vertical_offset.scale(v);
            let dir: Vector3<f64> = to - origin;
            let ray = Ray::new(origin, dir);
            let color: Rgb<u8> = cast_ray(&ray, &sphere_array, &light);
            img.put_pixel(i, j, color);
        }
    }

    img.save("test.png").unwrap();

}

fn cast_ray(ray: &Ray, array: &[Sphere], light: &Point3<f64>) -> Rgb<u8> {
    // start time at -1 to see if it changes later
    let mut hit_record: HitRecord = HitRecord::new(-1.0, Unit::new_normalize(Vector3::new(1.0, 0.0, 0.0)), Point3::origin());
    for sphere in array.iter() {
        let attempt: Option<HitRecord> = sphere.intersects(ray, 0.0, 10.0);
        match attempt {
            Some(x) => {
                if hit_record.t < 0.0 || x.t < hit_record.t {
                    hit_record = x;
                }
            }
            None => {}
        }
    }
    if hit_record.t < 0.0 { // miss
        return Rgb([55, 155, 255]);
    } else {
        let hit: Point3<f64> = hit_record.p;
        let normal: Unit<Vector3<f64>> = hit_record.n;
        let light_dir: Unit<Vector3<f64>> = Unit::new_normalize(hit - light);
        let brightness: f64 = light_dir.dot(normal.as_ref()).max(0.0);
        return Rgb([((255 as f64) * brightness) as u8, 0, 0]);
    }
    
}