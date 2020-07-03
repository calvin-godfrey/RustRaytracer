extern crate image;


use image::{Rgb, RgbImage};
use nalgebra::base::{Unit, Vector3};
use nalgebra::geometry::Point3;
use rand::prelude::*;
use rand::distributions::Standard;

static INFINITY: f64 = 1e308;
static PI: f64 = 3.14159265358979;
static SMALL: f64 = 0.001;

struct Camera {
    origin: Point3<f64>,
    upper_left_corner: Point3<f64>,
    horizontal_offset: Vector3<f64>,
    vertical_offset: Vector3<f64>,
}

impl Camera {
    fn new(aspect_ratio: f64) -> Self {
        let viewport_height = 2.0;
        let viewport_width = viewport_height * aspect_ratio;
        let focal_length = 1.0;

        let origin: Point3<f64> = Point3::origin();
        let horizontal_offset: Vector3<f64> = Vector3::new(viewport_width, 0.0, 0.0);
        let vertical_offset: Vector3<f64> = Vector3::new(0.0, viewport_height, 0.0);
        // this is the point in world space that represents the bottom left corner of the plane that is being projected onto
        let upper_left_corner: Point3<f64> = origin - horizontal_offset.scale(0.5) + vertical_offset.scale(0.5) - Vector3::new(0.0, 0.0, focal_length);
        return Self {origin, upper_left_corner, horizontal_offset, vertical_offset};
    }

    fn get_ray(&self, u: f64, v: f64) -> Ray {
        let to: Point3<f64> = self.upper_left_corner + self.horizontal_offset.scale(u) - self.vertical_offset.scale(v);
        let dir: Vector3<f64> = to - self.origin;
        return Ray::new(self.origin, dir);
    }
}

trait Hittable {
    fn intersects(&self, ray: &Ray, tmin: f64, tmax: f64) -> Option<HitRecord>;
}

struct HitRecord {
    t: f64, // time of hit along ray
    n: Unit<Vector3<f64>>, // normal of surface at point
    p: Point3<f64>, // point of intersection
    front: bool, // if the normal points outwards or not
}

impl HitRecord {
    fn new(t: f64, n: Unit<Vector3<f64>>, p: Point3<f64>, front: bool) -> Self { Self { t, n, p, front } }
    fn set_front(&mut self, ray: &Ray) {
        self.front = ray.dir.dot(self.n.as_ref()) < 0.0;
        self.n = if self.front { self.n } else { -self.n }
    }
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
            let mut hit_record = HitRecord::new(ans, Unit::new_normalize(hit - self.center), hit, true);
            hit_record.set_front(ray);
            return Some(hit_record);
        }
        let ans = (-b + root) * inv_a;
        if ans < tmax && ans > tmin {
            let hit = ray.at(ans);
            let mut hit_record = HitRecord::new(ans, Unit::new_normalize(hit - self.center), hit, true);
            hit_record.set_front(ray);
            return Some(hit_record);
        } else {
            return None;
        }
    }
}


fn main() {
    let image_width: u32 = 1920;
    let aspect_ratio = 16.0 / 9.0;
    let image_height: u32 = ((image_width as f64) / aspect_ratio).round() as u32;
    let camera: Camera = Camera::new(aspect_ratio);
    let samples_per_pixel = 500;
    let max_depth: u32 = 50;
    let mut img = RgbImage::new(image_width, image_height);

    let mut vec: Vec<Box<dyn Hittable>> = Vec::new();
    vec.push(Box::new(Sphere::new(Point3::new(0.0, 0.0, -1.0), 0.5)));
    vec.push(Box::new(Sphere::new(Point3::new(0.0, -100.5, -1.0), 100.)));
    let light: Point3<f64> = Point3::new(0.0, 0.0, 0.0);

    for i in 0u32..image_width {
        println!("Scanning row {} of {}", i, image_width);
        for j in 0u32..image_height {
            let mut color: Point3<f64> = Point3::origin();
            for _ in 0u32..samples_per_pixel {
                let u: f64 = (i as f64 + rand()) / ((image_width - 1) as f64);
                let v: f64 = (j as f64 + rand()) / ((image_height - 1) as f64);
                let ray = camera.get_ray(u, v);
                let res: Vector3<f64> = cast_ray(&ray, &vec, &light, max_depth);
                color = color + res;
            }
            draw_color(&mut img, i, j, &color, samples_per_pixel);
        }
    }

    img.save("test2.png").unwrap();

}

fn cast_ray(ray: &Ray, vec: &Vec<Box<dyn Hittable>>, light: &Point3<f64>, depth: u32) -> Vector3<f64> {
    if depth <= 0 {
        return Vector3::new(0.0, 0.0, 0.0);
    }
    let mut hit_record: HitRecord = HitRecord::new(INFINITY, Unit::new_normalize(Vector3::new(1.0, 0.0, 0.0)), Point3::origin(), true);
    for sphere in vec.iter() {
        let attempt: Option<HitRecord> = sphere.intersects(ray, SMALL, INFINITY);
        match attempt {
            Some(x) => {
                if x.t < hit_record.t {
                    hit_record = x;
                }
            }
            None => {}
        }
    }
    if hit_record.t == INFINITY { // miss
        let white = Rgb([255u8, 255u8, 255u8]);
        let blue = Rgb([50u8, 129u8, 255u8]);
        let unit: Unit<Vector3<f64>> = Unit::new_normalize(ray.dir);
        let color = gradient(&white, &blue, 0.5 * (1.0 + unit.as_ref().y));
        return Vector3::new(color[0] as f64, color[1] as f64, color[2] as f64);
    } else {
        let new_target: Point3<f64> = hit_record.p + hit_record.n.as_ref() + rand_in_unit_sphere();
        let new_ray = Ray::new(hit_record.p, new_target - hit_record.p);
        cast_ray(&new_ray, vec, light, depth - 1).scale(0.5)
    }
    
}

fn gradient(from: &Rgb<u8>, to: &Rgb<u8>, scale: f64) -> Rgb<u8> {
    let r: u8 = ((1.0 - scale) * from[0] as f64 + (scale * (to[0] as f64))) as u8;
    let g: u8 = ((1.0 - scale) * from[1] as f64 + (scale * (to[1] as f64))) as u8;
    let b: u8 = ((1.0 - scale) * from[2] as f64 + (scale * (to[2] as f64))) as u8;
    Rgb([r, g, b])
}

fn clamp(x: f64, min: f64, max: f64) -> f64 {
    if x < min {
        return min;
    } else if x > max {
        return max;
    }
    return x;
}

fn rand() -> f64 {
    StdRng::from_entropy().sample(Standard)
}

fn rand_range(min: f64, max: f64) -> f64 {
    return min + (max - min) * rand();
}

fn draw_color(img: &mut RgbImage, i: u32, j: u32, color: &Point3<f64>, samples: u32) {
    let r = color.x;
    let g = color.y;
    let b = color.z;
    let scale = 1. / (samples as f64 * 255.);
    let r = r * scale;
    let g = g * scale;
    let b = b * scale;
    let ans = Rgb([(r.sqrt() * 255.).round() as u8,
                            (g.sqrt() * 255.).round() as u8,
                            (b.sqrt() * 255.).round() as u8]);
    img.put_pixel(i, j, ans);
}

fn rand_in_unit_sphere() -> Vector3<f64> {
    let a = rand_range(0.,2. * PI);
    let z = rand_range(-1., 1.);
    let r = (1. - z * z).sqrt();
    return Vector3::new(r * a.cos(), r * a.sin(), z);
}