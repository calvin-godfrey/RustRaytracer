use nalgebra::base::{Matrix4, Vector3};
use nalgebra::geometry::{Point3, Rotation3, Projective3};
use std::sync::Arc;
use crate::hittable::*;
use crate::consts::*;
use crate::material::materials::{Texture, Material};
use crate::util;
use crate::geometry;
use crate::primitive::Primitive;

#[allow(dead_code)]
pub fn make_world() -> (geometry::Camera, BvhNode) {
    let from: Point3<f64> = Point3::new(13., 2.,3.);
    let to: Point3<f64> = Point3::new(0., 0., 0.0);
    let up: Vector3<f64> = Vector3::new(0., 1., 0.);

    let camera = geometry::Camera::new(from, to, up, ASPECT_RATIO, 20., 0.16, 10.);

    let mut world: Vec<Box<Primitive>> = Vec::new();
    let ground = Box::new(Primitive::new_sphere(Point3::new(0., -10000., 0.), 10000., Arc::new(Material::new_lambertian(Texture::new_checkered(Box::new(Texture::new_solid_color(Vector3::new(201., 201., 201.))), Box::new(Texture::new_solid_color(Vector3::new(51., 75.5, 25.5))), 4.)))));
    // let ground = Box::new(Primitive::new_sphere(Point3::new(0., -10000., 0.), 10000., Arc::new(Material::new_lambertian(Texture::new_perlin(4.)))));

    world.push(ground);

    for a in -11..11 {
        for b in -11..11 {
            let center = Point3::new(a as f64 + 0.9*util::rand(), 0.2, b as f64 + 0.9 * util::rand());
            if (center - Point3::new(4., 0.2, 0.)).norm() > 0.9 {
                let mat = util::rand();
                if mat < 0.6 { // matte
                    let color: Vector3<f64> = Vector3::new(255. * util::rand() * util::rand_range(0.4, 0.8), 255. * util::rand() * util::rand_range(0.4, 0.8), 255. * util::rand() * util::rand_range(0.4, 0.8));
                    // let texture = Texture::new_texture("data/cat/Cat_diffuse.jpg");
                    world.push(Box::new(Primitive::new_sphere(center, 0.2, Arc::new(Material::new_lambertian(Texture::new_solid_color(color))))));
                } else if mat < 0.86 { // metal
                    let color: Vector3<f64> = Vector3::new(util::rand_range(30., 200.), util::rand_range(30., 200.), util::rand_range(30., 200.));
                    world.push(Box::new(Primitive::new_sphere(center, 0.2, Arc::new(Material::new_metal(color, util::rand_range(0., 0.5))))));
                } else { // glass
                    world.push(Box::new(Primitive::new_sphere(center, 0.2, Arc::new(Material::new_dielectric(util::rand_range(1.2, 1.8))))));
                }
            }
        }
    }

    world.push(Box::new(Primitive::new_sphere(Point3::new(0., 1., 0.), 1., Arc::new(Material::new_dielectric(1.5)))));
    world.push(Box::new(Primitive::new_sphere(Point3::new(-4., 1., 0.), 1., Arc::new(Material::new_lambertian(Texture::new_perlin(6.))))));
    // world.push(Box::new(Primitive::new_sphere(Point3::new(4., 1., 0.), 1., Arc::new(Material::new_metal(Vector3::new(178.5, 153., 127.5), 0.05)))));

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
    let ground = Box::new(Primitive::new_sphere(Point3::new(0., -10000., 0.), 10000., Arc::new(Material::new_lambertian(Texture::new_checkered(Box::new(Texture::new_solid_color(Vector3::new(201., 201., 201.))), Box::new(Texture::new_solid_color(Vector3::new(51., 75.5, 25.5))), 10.)))));
    world.push(ground);

    for a in -11..11 {
        for b in -11..11 {
            let center = Point3::new(a as f64 + 0.9*util::rand(), 0.2, b as f64 + 0.9 * util::rand());
            if (center - Point3::new(4., 0.2, 0.)).norm() > 0.9 {
                let mat = util::rand();
                if mat < 0.75 { // matte
                    let center2 = center + Vector3::new(0., util::rand_range(0., 0.5), 0.);
                    let color: Vector3<f64> = Vector3::new(255. * util::rand() * util::rand_range(0.4, 0.8), 255. * util::rand() * util::rand_range(0.4, 0.8), 255. * util::rand() * util::rand_range(0.4, 0.8));
                    world.push(Box::new(Primitive::new_moving_sphere(center, center2, 0., 1., 0.2, Arc::new(Material::new_lambertian(Texture::new_solid_color(color))))));
                } else if mat < 0.92 { // metal
                    let color: Vector3<f64> = Vector3::new(util::rand_range(30., 200.), util::rand_range(30., 200.), util::rand_range(30., 200.));
                    world.push(Box::new(Primitive::new_sphere(center, 0.2, Arc::new(Material::new_metal(color, util::rand_range(0., 0.5))))));
                } else { // glass
                    world.push(Box::new(Primitive::new_sphere(center, 0.2, Arc::new(Material::new_dielectric(util::rand_range(1.2, 1.8))))));
                }
            }
        }
    }

    world.push(Box::new(Primitive::new_sphere(Point3::new(0., 1., 0.), 1., Arc::new(Material::new_dielectric(1.5)))));
    world.push(Box::new(Primitive::new_sphere(Point3::new(-4., 1., 0.), 1., Arc::new(Material::new_lambertian(Texture::new_solid_color(Vector3::new(102., 51., 25.)))))));
    world.push(Box::new(Primitive::new_sphere(Point3::new(4., 1., 0.), 1., Arc::new(Material::new_metal(Vector3::new(178.5, 153., 127.5), 0.05)))));

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

    let mesh = Mesh::new("data/cat/cat.obj", Projective3::from_matrix_unchecked(transform));
    let arc_mesh = Arc::new(mesh);
    let triangles = Mesh::generate_triangles(&arc_mesh);
    let sphere = Primitive::new_sphere(Point3::new(70., -20., 10.), 50., Arc::new(Material::new_metal(Vector3::new(178.5, 153., 127.5), 0.)));
    let other = Primitive::new_sphere(Point3::new(0., -1000., 0.), 950., Arc::new(Material::new_metal(Vector3::new(128., 153., 150.), 0.1)));

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

#[allow(dead_code)]
pub fn basic_light() -> (geometry::Camera, BvhNode) {
    let from: Point3<f64> = Point3::new(13., 2.,3.);
    let to: Point3<f64> = Point3::new(0., 2.5, 0.1);
    let up: Vector3<f64> = Vector3::new(0., 1., 0.);

    let camera = geometry::Camera::new_motion_blur(from, to, up, ASPECT_RATIO, 70., 0.0, 10., 0., 1.);
    let mut vec: Vec<Box<Primitive>> = Vec::new();
    let perlin = Texture::new_perlin(4.);
    let tex = Arc::new(Material::new_lambertian(perlin));
    vec.push(Box::new(Primitive::new_sphere(Point3::new(0., -1000., 0.), 1000., Arc::clone(&tex))));
    vec.push(Box::new(Primitive::new_sphere(Point3::new(0., 2., 0.), 2., Arc::clone(&tex))));

    let light = Arc::new(Material::new_diffuse(Texture::new_solid_color(Vector3::new(1000., 1000., 1000.))));
    vec.push(Box::new(Primitive::new_xy_rect(3., 1., 5., 3., -2., Arc::clone(&light))));
    vec.push(Box::new(Primitive::new_sphere(Point3::new(0., 7., 0.), 2., Arc::clone(&light))));

    let len = vec.len();
    let node = BvhNode::new(&mut vec, 0, len, 0., 0.);
    (camera, node)
}

#[allow(dead_code)]
pub fn cornell_box() -> (geometry::Camera, BvhNode) {

    let from = Point3::new(278., 278., -800.);
    let to = Point3::new(278., 278., 0.);
    let up = Vector3::new(0., 1., 0.);

    let camera = geometry::Camera::new_motion_blur(from, to, up, ASPECT_RATIO, 40., 0., 10., 0., 1.);

    let mut vec: Vec<Box<Primitive>> = Vec::new();
    let red   = Arc::new(Material::new_lambertian(Texture::new_solid_color(Vector3::new(165.75, 12.75, 12.75))));
    let white = Arc::new(Material::new_lambertian(Texture::new_solid_color(Vector3::new(186.15, 186.15, 186.15))));
    let green = Arc::new(Material::new_lambertian(Texture::new_solid_color(Vector3::new(30.6, 114.75, 38.25))));
    let light = Arc::new(Material::new_diffuse(Texture::new_solid_color(Vector3::new(3825., 3825., 3825.))));

    vec.push(Box::new(Primitive::new_flip_face(Box::new(Primitive::new_yz_rect(0., 0., 555., 555., 555., Arc::clone(&green))))));
    vec.push(Box::new(Primitive::new_yz_rect(0., 0., 555., 555., 0., Arc::clone(&red))));
    vec.push(Box::new(Primitive::new_xz_rect(213., 227., 343., 332., 554.9, Arc::clone(&light))));
    vec.push(Box::new(Primitive::new_xz_rect(0., 0., 555., 555., 0., Arc::clone(&white))));
    vec.push(Box::new(Primitive::new_flip_face(Box::new(Primitive::new_xz_rect(0., 0., 555., 555., 555., Arc::clone(&white))))));
    vec.push(Box::new(Primitive::new_flip_face(Box::new(Primitive::new_xy_rect(0., 0., 555., 555., 555., Arc::clone(&white))))));
    

    let first_translate = Projective3::from_matrix_unchecked(Matrix4::identity().append_translation(&Vector3::new(265., 0., 295.)));
    let second_translate = Projective3::from_matrix_unchecked(Matrix4::identity().append_translation(&Vector3::new(130., 0., 65.)));

    let r1 = Rotation3::from_euler_angles(0., 15. * PI / 180., 0.);
    let r2 = Rotation3::from_euler_angles(0., -18. * PI / 180., 0.);

    let first_transform = first_translate * r1;
    let second_transform = second_translate * r2;

    let cube1 = Cube::new_transform(Vector3::new(0., 0., 0.), Vector3::new(165., 165., 165.), Arc::clone(&white), Arc::new(second_transform));
    vec.extend(cube1.get_sides());
    let cube2 = Cube::new_transform(Vector3::new(0., 0., 0.), Vector3::new(165., 330., 165.), Arc::clone(&white), Arc::new(first_transform));
    vec.extend(cube2.get_sides());
    
    let len = vec.len();
    
    let node = BvhNode::new(&mut vec, 0, len, 0., 1.);
    (camera, node)
}