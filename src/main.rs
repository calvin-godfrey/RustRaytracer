use image::RgbImage;
use nalgebra::base::{Unit, Vector3, Matrix};
use nalgebra::geometry::Point3;

mod util;
mod consts;
use consts::*;

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
    fn new(from: Point3<f64>, to: Point3<f64>, up: Vector3<f64>, aspect_ratio: f64, vfov: f64) -> Self {

        let w: Unit<Vector3<f64>> = Unit::new_normalize(from - to);
        let u: Unit<Vector3<f64>> = Unit::new_normalize(Matrix::cross(&up, &w));
        let v: Unit<Vector3<f64>> = Unit::new_normalize(Matrix::cross(&w, &u));

        let theta = vfov * PI / 180.;
        let h = (theta / 2.).tan();
        let viewport_height = 2. * h;
        let viewport_width = viewport_height * aspect_ratio;

        let origin: Point3<f64> = from;
        let horizontal_offset: Vector3<f64> = u.scale(viewport_width);
        let vertical_offset: Vector3<f64> = v.scale(viewport_height);
        // this is the point in world space that represents the bottom left corner of the plane that is being projected onto
        let upper_left_corner: Point3<f64> = origin -
                                             horizontal_offset.scale(0.5) +
                                             vertical_offset.scale(0.5) -
                                             w.as_ref();
        
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

        let camera: Camera = Camera::new(Point3::new(-2., 2., 1.), Point3::new(0., 0., 0.7), Vector3::new(0., 1., 0.), aspect_ratio, 90.);
        let samples_per_pixel = 20;
        let max_depth: u32 = 25;
        let mut img = RgbImage::new(image_width, image_height);
        let path = "test2.png";
        let mut vec: Vec<Box<dyn Hittable>> = Vec::new();
        vec.push(Box::new(Sphere::new(Point3::new(-1., 0.0, -1.0), 0.5, Box::new(materials::Lambertian::new(Vector3::new(255.0, 90.0, 90.0))))));
        vec.push(Box::new(Sphere::new(Point3::new(0.0, -100.6, -1.0), 100., Box::new(materials::Lambertian::new(Vector3::new(90.0, 90.0, 255.0))))));
        vec.push(Box::new(Sphere::new(Point3::new(1., 0., -1.), 0.5, Box::new(materials::Metal::new(Vector3::new(205., 153., 51.), 0.3)))));
        vec.push(Box::new(Sphere::new(Point3::new(-0., 0., -1.), 0.5, Box::new(materials::Dielectric::new(1.35)))));
        let light: Point3<f64> = Point3::new(0.0, 0.0, 0.0);

        let cols: bool = true;

        if !cols {
            let mut pixels: Vec<Vec<(f64, f64, f64)>> = Vec::new();
            for _ in 0..image_height {
                let mut temp: Vec<(f64, f64, f64)> = Vec::new();
                for _ in 0..image_width {
                    temp.push((0., 0., 0.));
                }
                pixels.push(temp);
            }
            for r in 0u32..(image_width * image_height * samples_per_pixel) {
                if r % 100000 == 0 {
                    println!("Drawing ray {} of {}", r, image_width * image_height * samples_per_pixel);
                }
                if r % 1000000 == 0 {
                    util::draw_picture(&mut img, &pixels, path);
                    img.save(path).unwrap();
                }
                let u: f64 = util::rand();
                let v: f64 = util::rand();
                let ray = camera.get_ray(u, v);
                let res = cast_ray(&ray, &vec, &light, max_depth);
                let i = (u * image_width as f64).floor() as usize;
                let j = (v * image_height as f64).floor() as usize;
                util::increment_color(&mut pixels, j, i, &res, samples_per_pixel);
            }

        } else {
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
        }
        img.save(path).unwrap();
}