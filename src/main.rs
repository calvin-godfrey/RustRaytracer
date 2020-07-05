use image::RgbImage;
use nalgebra::base::{Unit, Vector3, Matrix};
use nalgebra::geometry::Point3;
use std::thread;
use std::sync::{Arc, Mutex};

mod util;
mod consts;
use consts::*;

mod material;
use material::materials;

mod hittable;
use hittable::{Hittable, Sphere};

mod geometry;
use geometry::{Ray};

#[derive(Copy, Clone)]
struct Camera {
    origin: Point3<f64>,
    upper_left_corner: Point3<f64>,
    horizontal_offset: Vector3<f64>,
    vertical_offset: Vector3<f64>,
    lens_radius: f64,
    u: Unit<Vector3<f64>>, // horizontal
    v: Unit<Vector3<f64>>, // vertical
    w: Unit<Vector3<f64>>, // direction of sight
}

impl Camera {
    fn new(from: Point3<f64>, to: Point3<f64>, up: Vector3<f64>, aspect_ratio: f64, vfov: f64, aperture: f64, focus_dist: f64) -> Self {
        let w: Unit<Vector3<f64>> = Unit::new_normalize(from - to);
        let u: Unit<Vector3<f64>> = Unit::new_normalize(Matrix::cross(&up, &w));
        let v: Unit<Vector3<f64>> = Unit::new_normalize(Matrix::cross(&w, &u));

        let theta = vfov * PI / 180.;
        let h = (theta / 2.).tan();
        let viewport_height = 2. * h;
        let viewport_width = viewport_height * aspect_ratio;

        let origin: Point3<f64> = from;
        let horizontal_offset: Vector3<f64> = u.scale(viewport_width * focus_dist);
        let vertical_offset: Vector3<f64> = v.scale(viewport_height * focus_dist);
        // this is the point in world space that represents the bottom left corner of the plane that is being projected onto
        let upper_left_corner: Point3<f64> = origin -
                                             horizontal_offset.scale(0.5) +
                                             vertical_offset.scale(0.5) -
                                             w.as_ref().scale(focus_dist);

        let lens_radius = aperture / 2.;
        
        return Self {origin, upper_left_corner, horizontal_offset, vertical_offset, lens_radius, u, v, w};
    }

    fn get_ray(&self, u: f64, v: f64) -> Ray {
        let in_disk: Vector3<f64> = util::rand_in_disk().scale(self.lens_radius);
        let offset = self.u.scale(in_disk.x) + self.v.scale(in_disk.y);

        let to: Point3<f64> = self.upper_left_corner + self.horizontal_offset.scale(u) - self.vertical_offset.scale(v);
        let dir: Vector3<f64> = to - self.origin;
        // have to subtract offset from the direction so that it points back to where it was originally supposed to
        return Ray::new(self.origin + offset, dir - offset);
    }
}

fn main() {
    let from: Point3<f64> = Point3::new(13., 2.,3.);
    let to: Point3<f64> = Point3::new(0., 0., 0.0);
    let up: Vector3<f64> = Vector3::new(0., 1., 0.);

    let camera: Camera = Camera::new(from, to, up, ASPECT_RATIO, 20., 0.16, 10.);
    let mut img = RgbImage::new(IMAGE_WIDTH, IMAGE_HEIGHT);
    let vec = make_world();
    if SINGLE_THREAD {
        singlethread(&mut img, PATH, &vec, &camera);
    } else {
        multithread(img, vec, camera);
    }
}

fn make_world() -> Vec<Box<dyn Hittable>> {
    let mut world: Vec<Box<dyn Hittable>> = Vec::new();
    let ground = Box::new(Sphere::new(Point3::new(0., -10000., 0.), 10000., Box::new(materials::Lambertian::new(Vector3::new(127., 127., 127.)))));
    world.push(ground);

    for a in -11..11 {
        for b in -11..11 {
            let center = Point3::new(a as f64 + 0.9*util::rand(), 0.2, b as f64 + 0.9 * util::rand());
            if (center - Point3::new(4., 0.2, 0.)).norm() > 0.9 {
                let mat = util::rand();
                if mat < 0.6 { // matte
                    let color: Vector3<f64> = Vector3::new(255. * util::rand() * util::rand_range(0.4, 0.8), 255. * util::rand() * util::rand_range(0.4, 0.8), 255. * util::rand() * util::rand_range(0.4, 0.8));
                    world.push(Box::new(Sphere::new(center, 0.2, Box::new(materials::Lambertian::new(color)))));
                } else if mat < 0.86 { // metal
                    let color: Vector3<f64> = Vector3::new(util::rand_range(30., 200.), util::rand_range(30., 200.), util::rand_range(30., 200.));
                    world.push(Box::new(Sphere::new(center, 0.2, Box::new(materials::Metal::new(color, util::rand_range(0., 0.5))))));
                } else { // glass
                    world.push(Box::new(Sphere::new(center, 0.2, Box::new(materials::Dielectric::new(util::rand_range(1.2, 1.8))))));
                }
            }
        }
    }

    world.push(Box::new(Sphere::new(Point3::new(0., 1., 0.), 1., Box::new(materials::Dielectric::new(1.5)))));
    world.push(Box::new(Sphere::new(Point3::new(-4., 1., 0.), 1., Box::new(materials::Lambertian::new(Vector3::new(102., 51., 25.))))));
    world.push(Box::new(Sphere::new(Point3::new(4., 1., 0.), 1., Box::new(materials::Metal::new(Vector3::new(178.5, 153., 127.5), 0.05)))));

    world
}

fn singlethread(img: &mut image::RgbImage, path: &str, vec: &Vec<Box<dyn Hittable>>, camera: &Camera) {
    if !COLS {
        let mut pixels: Vec<Vec<(f64, f64, f64, u32)>> = Vec::new();
        for _ in 0..IMAGE_HEIGHT {
            let mut temp: Vec<(f64, f64, f64, u32)> = Vec::new();
            for _ in 0..IMAGE_WIDTH {
                temp.push((0., 0., 0., 0u32));
            }
            pixels.push(temp);
        }
        let total_rays = IMAGE_WIDTH * IMAGE_HEIGHT * SAMPLES_PER_PIXEL;
        for r in 0..total_rays {
            if r % 100000 == 0 {
                println!("Drawing ray {} of {}, {:.2}%", r, total_rays, (r as f64 * 100.) / (total_rays as f64));
            }
            if r % 1000000 == 0 {
                util::draw_picture(img, &pixels, path);
                img.save(path).unwrap();
            }
            let u: f64 = util::rand();
            let v: f64 = util::rand();
            let ray = camera.get_ray(u, v);
            let res = geometry::cast_ray(&ray, &vec, MAX_DEPTH);
            let i = (u * IMAGE_WIDTH as f64).floor() as usize;
            let j = (v * IMAGE_HEIGHT as f64).floor() as usize;
            util::increment_color(&mut pixels, j, i, &res);
        }

    } else {
        for i in 0u32..IMAGE_WIDTH {
            println!("Scanning row {} of {}", i, IMAGE_WIDTH);
            for j in 0u32..IMAGE_HEIGHT {
                let mut color: Point3<f64> = Point3::origin();
                for _ in 0u32..SAMPLES_PER_PIXEL {
                    let u: f64 = (i as f64 + util::rand()) / ((IMAGE_WIDTH - 1) as f64);
                    let v: f64 = (j as f64 + util::rand()) / ((IMAGE_HEIGHT - 1) as f64);
                    let ray = camera.get_ray(u, v);
                    let res: Vector3<f64> = geometry::cast_ray(&ray, &vec, MAX_DEPTH);
                    color = color + res;
                }
                util::draw_color(img, i, j, &color, SAMPLES_PER_PIXEL);
            }
            if i % 100 == 0 {
                img.save(path).unwrap();
            }
        }
    }
    img.save(path).unwrap();
}

fn multithread (img: image::RgbImage, vec: Vec<Box<dyn Hittable>>, camera: Camera) {
    let mut pixels: Vec<Vec<(f64, f64, f64, u32)>> = Vec::new();
    for _ in 0..IMAGE_HEIGHT {
        let mut temp: Vec<(f64, f64, f64, u32)> = Vec::new();
        for _ in 0..IMAGE_WIDTH {
            temp.push((0., 0., 0., 0u32));
        }
        pixels.push(temp);
    }
    let pixels_mutex: Arc<Mutex<Vec<Vec<(f64, f64, f64, u32)>>>> = Arc::new(Mutex::new(pixels));
    let vec_arc: Arc<Vec<Box<dyn Hittable>>> = Arc::new(vec);

    let image_arc: Arc<Mutex<image::RgbImage>> = Arc::new(Mutex::new(img));

    let mut thread_vec: Vec<thread::JoinHandle<()>> = Vec::new();

    for thread_num in 0..NUM_THREADS {
        let camera_clone = camera.clone();
        let vec_clone = Arc::clone(&vec_arc);
        let pixels_clone = Arc::clone(&pixels_mutex);
        let image_clone = Arc::clone(&image_arc);

        thread_vec.push(thread::spawn(move || {
            for r in 0..RAYS_PER_THREAD {
                if r % THREAD_UPDATE == 0 {
                    println!("Thread {} drawing ray {} of {} ({:.2}%)", thread_num, r, RAYS_PER_THREAD, (r as f64 * 100.) / (RAYS_PER_THREAD as f64));
                    if thread_num == 0 { // only draw once
                        util::thread_safe_draw_picture(&image_clone, &pixels_clone, PATH);
                    }
                }
                let u: f64 = util::rand();
                let v: f64 = util::rand();
                let ray = camera_clone.get_ray(u, v);
                let res = geometry::cast_ray(&ray, &vec_clone, MAX_DEPTH);
                let i = (u * IMAGE_WIDTH as f64).floor() as usize;
                let j = (v * IMAGE_HEIGHT as f64).floor() as usize;
                util::thread_safe_increment_color(&pixels_clone, j, i, &res);
            }
        }));
    }

    for handle in thread_vec {
        handle.join().unwrap();
    }

    image_arc.lock().unwrap().save(PATH).unwrap();

    // for r in 0..total_rays {
    //     if r % 100000 == 0 {
    //         println!("Drawing ray {} of {}, {:.2}%", r, total_rays, (r as f64 * 100.) / (total_rays as f64));
    //     }
    //     if r % 1000000 == 0 {
    //         util::draw_picture(img, &pixels, path);
    //         img.save(path).unwrap();
    //     }
    //     let u: f64 = util::rand();
    //     let v: f64 = util::rand();
    //     let ray = camera.get_ray(u, v);
    //     let res = cast_ray(&ray, &vec, MAX_DEPTH);
    //     let i = (u * IMAGE_WIDTH as f64).floor() as usize;
    //     let j = (v * IMAGE_HEIGHT as f64).floor() as usize;
    //     util::increment_color(&mut pixels, j, i, &res);
    // }
}