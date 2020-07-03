use image::RgbImage;
use nalgebra::base::Vector3;
use nalgebra::geometry::Point3;

mod util;
mod consts;

mod material;
use material::materials;

mod hittable;
use hittable::{Hittable, Sphere};

mod geometry;
use geometry::{Ray, cast_ray};

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

fn main() {
    let image_width: u32 = 1920;
    let aspect_ratio: f64 = 16.0 / 9.0;
    let image_height: u32 = ((image_width as f64) / aspect_ratio).round() as u32;
    let camera: Camera = Camera::new(aspect_ratio);
    let samples_per_pixel = 200;
    let max_depth: u32 = 25;
    let mut img = RgbImage::new(image_width, image_height);
    let path = "test2.png";

    let mut vec: Vec<Box<dyn Hittable>> = Vec::new();
    vec.push(Box::new(Sphere::new(Point3::new(0.0, 0.0, -1.0), 0.5, Box::new(materials::Lambertian::new(Vector3::new(255.0, 90.0, 90.0))))));
    vec.push(Box::new(Sphere::new(Point3::new(0.0, -100.5, -1.0), 100., Box::new(materials::Lambertian::new(Vector3::new(90.0, 90.0, 255.0))))));
    vec.push(Box::new(Sphere::new(Point3::new(1., 0., -1.), 0.5, Box::new(materials::Metal::new(Vector3::new(205., 153., 51.), 0.3)))));
    vec.push(Box::new(Sphere::new(Point3::new(-1., 0., -1.), 0.5, Box::new(materials::Metal::new(Vector3::new(205., 205., 205.), 0.9)))));
    let light: Point3<f64> = Point3::new(0.0, 0.0, 0.0);

    // for r in 0u32..(image_width * image_height * samples_per_pixel) {
    //     if r % 100000 == 0 {
    //         println!("Drawing ray {} of {}", r, image_width * image_height * samples_per_pixel);
    //     }
    //     if r % 1000000 == 0 {
    //         img.save(path).unwrap();
    //     }
    //     let u: f64 = rand();
    //     let v: f64 = rand();
    //     let ray = camera.get_ray(u, v);
    //     let res = cast_ray(&ray, &vec, &light, max_depth);
    //     let i: u32 = (u * image_width as f64).floor() as u32;
    //     let j: u32 = (v * image_height as f64).floor() as u32;
    //     increment_color(&mut img, i, j, &res, samples_per_pixel);
    // }

    for i in 0u32..image_width {
        println!("Scanning row {} of {}", i, image_width);
        for j in 0u32..image_height {
            let mut color: Point3<f64> = Point3::origin();
            for _ in 0u32..samples_per_pixel {
                let u: f64 = (i as f64 + util::rand()) / ((image_width - 1) as f64);
                let v: f64 = (j as f64 + util::rand()) / ((image_height - 1) as f64);
                let ray = camera.get_ray(u, v);
                let res: Vector3<f64> = cast_ray(&ray, &vec, &light, max_depth);
                color = color + res;
            }
            util::draw_color(&mut img, i, j, &color, samples_per_pixel);
        }
        if i % 100 == 0 {
            img.save(path).unwrap();
        }
    }
    img.save(path).unwrap();
}