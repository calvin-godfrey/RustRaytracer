use nalgebra::base::{Matrix4, Vector3};
use nalgebra::geometry::{Point3, Rotation3};
use std::sync::Arc;
use crate::hittable::*;
use crate::consts::*;
use crate::materials;
use crate::util;
use crate::geometry;

#[allow(dead_code)]
pub fn make_world() -> (geometry::Camera, BvhNode) {
    let from: Point3<f64> = Point3::new(13., 2.,3.);
    let to: Point3<f64> = Point3::new(0., 0., 0.0);
    let up: Vector3<f64> = Vector3::new(0., 1., 0.);

    let camera = geometry::Camera::new(from, to, up, ASPECT_RATIO, 20., 0.16, 10.);

    let mut world: Vec<Box<Primitive>> = Vec::new();
    let ground = Box::new(make_sphere(Point3::new(0., -10000., 0.), 10000., Arc::new(materials::Material::new_lambertian(Vector3::new(127., 127., 127.)))));
    world.push(ground);

    for a in -11..11 {
        for b in -11..11 {
            let center = Point3::new(a as f64 + 0.9*util::rand(), 0.2, b as f64 + 0.9 * util::rand());
            if (center - Point3::new(4., 0.2, 0.)).norm() > 0.9 {
                let mat = util::rand();
                if mat < 0.6 { // matte
                    let color: Vector3<f64> = Vector3::new(255. * util::rand() * util::rand_range(0.4, 0.8), 255. * util::rand() * util::rand_range(0.4, 0.8), 255. * util::rand() * util::rand_range(0.4, 0.8));
                    world.push(Box::new(make_sphere(center, 0.2, Arc::new(materials::Material::new_lambertian(color)))));
                } else if mat < 0.86 { // metal
                    let color: Vector3<f64> = Vector3::new(util::rand_range(30., 200.), util::rand_range(30., 200.), util::rand_range(30., 200.));
                    world.push(Box::new(make_sphere(center, 0.2, Arc::new(materials::Material::new_metal(color, util::rand_range(0., 0.5))))));
                } else { // glass
                    world.push(Box::new(make_sphere(center, 0.2, Arc::new(materials::Material::new_dielectric(util::rand_range(1.2, 1.8))))));
                }
            }
        }
    }

    world.push(Box::new(make_sphere(Point3::new(0., 1., 0.), 1., Arc::new(materials::Material::new_dielectric(1.5)))));
    world.push(Box::new(make_sphere(Point3::new(-4., 1., 0.), 1., Arc::new(materials::Material::new_lambertian(Vector3::new(102., 51., 25.))))));
    world.push(Box::new(make_sphere(Point3::new(4., 1., 0.), 1., Arc::new(materials::Material::new_metal(Vector3::new(178.5, 153., 127.5), 0.05)))));

    let len = world.len();
    let node = BvhNode::new(&mut world, 0, len, 0., 0.);

    (camera, node)
}

#[allow(dead_code)]
pub fn make_world_moving() -> (geometry::Camera, BvhNode) {
    let from: Point3<f64> = Point3::new(13., 2.,3.);
    let to: Point3<f64> = Point3::new(0., 0., 0.0);
    let up: Vector3<f64> = Vector3::new(0., 1., 0.);

    let camera = geometry::Camera::new_motion_blur(from, to, up, ASPECT_RATIO, 20., 0.16, 10., 0., 1.);

    let mut world: Vec<Box<Primitive>> = Vec::new();
    let ground = Box::new(make_sphere(Point3::new(0., -10000., 0.), 10000., Arc::new(materials::Material::new_lambertian(Vector3::new(127., 127., 127.)))));
    world.push(ground);

    for a in -11..11 {
        for b in -11..11 {
            let center = Point3::new(a as f64 + 0.9*util::rand(), 0.2, b as f64 + 0.9 * util::rand());
            if (center - Point3::new(4., 0.2, 0.)).norm() > 0.9 {
                let mat = util::rand();
                if mat < 0.75 { // matte
                    let center2 = center + Vector3::new(0., util::rand_range(0., 0.5), 0.);
                    let color: Vector3<f64> = Vector3::new(255. * util::rand() * util::rand_range(0.4, 0.8), 255. * util::rand() * util::rand_range(0.4, 0.8), 255. * util::rand() * util::rand_range(0.4, 0.8));
                    world.push(Box::new(make_moving_sphere(center, center2, 0., 1., 0.2, Arc::new(materials::Material::new_lambertian(color)))));
                } else if mat < 0.92 { // metal
                    let color: Vector3<f64> = Vector3::new(util::rand_range(30., 200.), util::rand_range(30., 200.), util::rand_range(30., 200.));
                    world.push(Box::new(make_sphere(center, 0.2, Arc::new(materials::Material::new_metal(color, util::rand_range(0., 0.5))))));
                } else { // glass
                    world.push(Box::new(make_sphere(center, 0.2, Arc::new(materials::Material::new_dielectric(util::rand_range(1.2, 1.8))))));
                }
            }
        }
    }

    world.push(Box::new(make_sphere(Point3::new(0., 1., 0.), 1., Arc::new(materials::Material::new_dielectric(1.5)))));
    world.push(Box::new(make_sphere(Point3::new(-4., 1., 0.), 1., Arc::new(materials::Material::new_lambertian(Vector3::new(102., 51., 25.))))));
    world.push(Box::new(make_sphere(Point3::new(4., 1., 0.), 1., Arc::new(materials::Material::new_metal(Vector3::new(178.5, 153., 127.5), 0.05)))));

    let len = world.len();
    let node = BvhNode::new(&mut world, 0, len, 0., 0.);

    (camera, node)
}

#[allow(dead_code)]
pub fn sphere_cat_bvh() -> (geometry::Camera, BvhNode) {
    let from = Point3::new(-200. * 0.3, -100. * 0.3, 100. * 0.6);
    let to = Point3::new(50., -35., 0.);
    let up = Vector3::new(0., 1., 0.);
    let camera = geometry::Camera::new(from, to, up, ASPECT_RATIO, 50., 0.0, 10.);

    let transform: Matrix4<f64> = Matrix4::identity().append_translation(&Vector3::new(0., -50., 0.));
    let rot: Matrix4<f64> = Rotation3::from_euler_angles(-PI / 2., 0., 0.).to_homogeneous();
    let transform = transform * rot;

    let mesh = Mesh::new("data/cat/cat.obj", transform);
    let arc_mesh = Arc::new(mesh);
    let triangles = Mesh::generate_triangles(&arc_mesh);
    let sphere = make_sphere(Point3::new(70., -20., 10.), 50., Arc::new(materials::Material::new_metal(Vector3::new(178.5, 153., 127.5), 0.)));
    let other = make_sphere(Point3::new(0., -1000., 0.), 950., Arc::new(materials::Material::new_metal(Vector3::new(128., 153., 150.), 0.1)));

    let mut vec: Vec<Box<Primitive>> = Vec::new();
    vec.push(Box::new(sphere));
    vec.push(Box::new(other));
    for tri in triangles {
        vec.push(tri);
    }
    let len = vec.len();
    let node: BvhNode = BvhNode::new(&mut vec, 0, len, 0., 0.);
    println!("Made bvh");
    (camera, node)
}