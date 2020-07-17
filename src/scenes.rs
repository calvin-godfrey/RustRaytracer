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
pub fn make_world() -> (geometry::Camera, BvhNode, Vec<Primitive>, Vec<Material>, Vec<Texture>) {
    let from: Point3<f64> = Point3::new(13., 2.,3.);
    let to: Point3<f64> = Point3::new(0., 0., 0.0);
    let up: Vector3<f64> = Vector3::new(0., 1., 0.);

    let camera = geometry::Camera::new(from, to, up, ASPECT_RATIO, 20., 0.16, 10.);

    let mut world: Vec<Primitive> = Vec::new();
    let mut textures: Vec<Texture> = Vec::new();
    let mut materials: Vec<Material> = Vec::new();
    textures.push(Texture::new_checkered(Box::new(Texture::new_solid_color(Vector3::new(201., 201., 201.))), Box::new(Texture::new_solid_color(Vector3::new(51., 75.5, 25.5))), 4.));
    materials.push(Material::new_lambertian(0));
    let ground = Primitive::new_sphere(Point3::new(0., -10000., 0.), 10000., 0);
    // let ground = Primitive::new_sphere(Point3::new(0., -10000., 0.), 10000., Arc::new(Material::new_lambertian(Texture::new_perlin(4.))));

    world.push(ground);

    for a in -11..11 {
        for b in -11..11 {
            let center = Point3::new(a as f64 + 0.9*util::rand(), 0.2, b as f64 + 0.9 * util::rand());
            if (center - Point3::new(4., 0.2, 0.)).norm() > 0.9 {
                let mat = util::rand();
                if mat < 0.6 { // matte
                    let color: Vector3<f64> = Vector3::new(255. * util::rand() * util::rand_range(0.4, 0.8), 255. * util::rand() * util::rand_range(0.4, 0.8), 255. * util::rand() * util::rand_range(0.4, 0.8));
                    let texture = Texture::new_solid_color(color);
                    textures.push(texture);
                    let mat = Material::new_lambertian(textures.len() - 1);
                    materials.push(mat);
                    world.push(Primitive::new_sphere(center, 0.2, materials.len() - 1));
                } else if mat < 0.86 { // metal
                    let color: Vector3<f64> = Vector3::new(util::rand_range(30., 200.), util::rand_range(30., 200.), util::rand_range(30., 200.));
                    let mat = Material::new_metal(color, util::rand_range(0., 0.5));
                    materials.push(mat);
                    world.push(Primitive::new_sphere(center, 0.2, materials.len() - 1));
                } else { // glass
                    let mat = Material::new_dielectric(util::rand_range(1.2, 1.8));
                    materials.push(mat);
                    world.push(Primitive::new_sphere(center, 0.2, materials.len() - 1));
                }
            }
        }
    }

    let new_texture = Texture::new_perlin(6.);
    textures.push(new_texture);
    let perlin_mat = Material::new_lambertian(textures.len() - 1);
    materials.push(perlin_mat);
    world.push(Primitive::new_sphere(Point3::new(-4., 1., 0.), 1., materials.len() - 1));
    materials.push(Material::new_dielectric(1.2));
    world.push(Primitive::new_sphere(Point3::new(0., 1., 0.), 1., materials.len() - 1));

    // world.push(Primitive::new_sphere(Point3::new(4., 1., 0.), 1., Arc::new(Material::new_metal(Vector3::new(178.5, 153., 127.5), 0.05))));

    let len = world.len();
    let node = BvhNode::new(&mut world, 0, len, 0., 0.);

    (camera, node, world, materials, textures)
}

#[allow(dead_code)]
pub fn make_world_moving() -> (geometry::Camera, BvhNode, Vec<Primitive>, Vec<Material>, Vec<Texture>) {
    let from: Point3<f64> = Point3::new(13., 2.,3.);
    let to: Point3<f64> = Point3::new(0., 0., 0.0);
    let up: Vector3<f64> = Vector3::new(0., 1., 0.);

    let camera = geometry::Camera::new_motion_blur(from, to, up, ASPECT_RATIO, 20., 0.16, 10., 0., 1.);

    let mut world: Vec<Primitive> = Vec::new();
    let mut materials: Vec<Material> = Vec::new();
    let mut textures: Vec<Texture> = Vec::new();
    
    let checker_text = Texture::new_checkered(Box::new(Texture::new_solid_color(Vector3::new(201., 201., 201.))), Box::new(Texture::new_solid_color(Vector3::new(51., 75.5, 25.5))), 10.);
    textures.push(checker_text);
    let checker_mat = Material::new_lambertian(0);
    materials.push(checker_mat);
    let ground = Primitive::new_sphere(Point3::new(0., -10000., 0.), 10000., 0);
    world.push(ground);

    for a in -11..11 {
        for b in -11..11 {
            let center = Point3::new(a as f64 + 0.9*util::rand(), 0.2, b as f64 + 0.9 * util::rand());
            if (center - Point3::new(4., 0.2, 0.)).norm() > 0.9 {
                let mat = util::rand();
                if mat < 0.75 { // matte
                    let center2 = center + Vector3::new(0., util::rand_range(0., 0.5), 0.);
                    let color: Vector3<f64> = Vector3::new(255. * util::rand() * util::rand_range(0.4, 0.8), 255. * util::rand() * util::rand_range(0.4, 0.8), 255. * util::rand() * util::rand_range(0.4, 0.8));
                    let texture = Texture::new_solid_color(color);
                    textures.push(texture);
                    let mat = Material::new_lambertian(textures.len() - 1);
                    materials.push(mat);
                    world.push(Primitive::new_moving_sphere(center, center2, 0., 1., 0.2, materials.len() - 1));
                } else if mat < 0.92 { // metal
                    let color: Vector3<f64> = Vector3::new(util::rand_range(30., 200.), util::rand_range(30., 200.), util::rand_range(30., 200.));
                    let mat = Material::new_metal(color, util::rand_range(0., 0.5));
                    materials.push(mat);
                    world.push(Primitive::new_sphere(center, 0.2, materials.len() - 1));
                } else { // glass
                    let mat = Material::new_dielectric(util::rand_range(1.2, 1.8));
                    materials.push(mat);
                    world.push(Primitive::new_sphere(center, 0.2, materials.len() - 1));
                }
            }
        }
    }

    let die = Material::new_dielectric(1.5);
    materials.push(die);
    world.push(Primitive::new_sphere(Point3::new(0., 1., 0.), 1., materials.len() - 1));
    let solid = Texture::new_solid_color(Vector3::new(102., 51., 25.));
    textures.push(solid);
    let solid_mat = Material::new_lambertian(textures.len() - 1);
    materials.push(solid_mat);
    world.push(Primitive::new_sphere(Point3::new(-4., 1., 0.), 1., materials.len() - 1));
    let last_metal = Material::new_metal(Vector3::new(178.5, 153., 127.5), 0.05);
    materials.push(last_metal);
    world.push(Primitive::new_sphere(Point3::new(4., 1., 0.), 1., materials.len() - 1));

    let len = world.len();
    let node = BvhNode::new(&mut world, 0, len, 0., 0.);

    (camera, node, world, materials, textures)
}

#[allow(dead_code)]
pub fn sphere_cat_bvh() -> (geometry::Camera, BvhNode, Vec<Primitive>, Vec<Material>, Vec<Texture>) {
    let from = Point3::new(-200. * 0.3, -100. * 0.3, 100. * 0.6);
    let to = Point3::new(50., -35., 0.);
    let up = Vector3::new(0., 1., 0.);
    let camera = geometry::Camera::new(from, to, up, ASPECT_RATIO, 50., 0.0, 10.);

    let transform: Matrix4<f64> = Matrix4::identity().append_translation(&Vector3::new(0., -50., 0.));
    let rot: Matrix4<f64> = Rotation3::from_euler_angles(-PI / 2., 0., 0.).to_homogeneous();
    let transform = transform * rot;

    let mut world: Vec<Primitive> = Vec::new();
    let mut materials: Vec<Material> = Vec::new();
    let mut textures: Vec<Texture> = Vec::new();

    let mesh = Mesh::new(&mut materials, &mut textures, "data/cat/cat.obj", Projective3::from_matrix_unchecked(transform));
    let arc_mesh = Arc::new(mesh);
    let triangles = Mesh::generate_triangles(&arc_mesh);

    materials.push(Material::new_metal(Vector3::new(178.5, 153., 127.5), 0.));
    materials.push(Material::new_metal(Vector3::new(128., 153., 150.), 0.1));

    let sphere = Primitive::new_sphere(Point3::new(70., -20., 10.), 50., 1);
    let other = Primitive::new_sphere(Point3::new(0., -1000., 0.), 950., 2);

    world.push(sphere);
    world.push(other);
    for tri in triangles {
        world.push(tri);
    }
    let len = world.len();
    let node: BvhNode = BvhNode::new(&mut world, 0, len, 0., 0.);
    (camera, node, world, materials, textures)
}

#[allow(dead_code)]
pub fn cornell_box() -> (geometry::Camera, BvhNode, Vec<Primitive>, Vec<Material>, Vec<Texture>) {

    let from = Point3::new(278., 278., -800.);
    let to = Point3::new(278., 278., 0.);
    let up = Vector3::new(0., 1., 0.);

    let camera = geometry::Camera::new_motion_blur(from, to, up, ASPECT_RATIO, 40., 0., 10., 0., 1.);

    let mut vec: Vec<Primitive> = Vec::new();
    let mut materials: Vec<Material> = Vec::new();
    let mut textures: Vec<Texture> = Vec::new();
    textures.push(Texture::new_solid_color(Vector3::new(165.75, 12.75, 12.75))); // red
    textures.push(Texture::new_solid_color(Vector3::new(186.15, 186.15, 186.15))); // white
    textures.push(Texture::new_solid_color(Vector3::new(30.6, 114.75, 38.25))); // green
    textures.push(Texture::new_solid_color(Vector3::new(3825., 3825., 3825.))); // light

    materials.push(Material::new_lambertian(0));
    materials.push(Material::new_lambertian(1));
    materials.push(Material::new_lambertian(2));
    materials.push(Material::new_diffuse(3));
    
    vec.push(Primitive::new_flip_face(Box::new(Primitive::new_yz_rect(0., 0., 555., 555., 555., 2))));
    vec.push(Primitive::new_yz_rect(0., 0., 555., 555., 0., 0));
    vec.push(Primitive::new_xz_rect(213., 227., 343., 332., 554.9, 3));
    vec.push(Primitive::new_xz_rect(0., 0., 555., 555., 0., 1));
    vec.push(Primitive::new_flip_face(Box::new(Primitive::new_xz_rect(0., 0., 555., 555., 555., 1))));
    vec.push(Primitive::new_flip_face(Box::new(Primitive::new_xy_rect(0., 0., 555., 555., 555., 1))));
    

    let first_translate = Projective3::from_matrix_unchecked(Matrix4::identity().append_translation(&Vector3::new(265., 0., 295.)));
    let second_translate = Projective3::from_matrix_unchecked(Matrix4::identity().append_translation(&Vector3::new(130., 0., 65.)));

    let r1 = Rotation3::from_euler_angles(0., 15. * PI / 180., 0.);
    let r2 = Rotation3::from_euler_angles(0., -18. * PI / 180., 0.);

    let first_transform = first_translate * r1;
    let second_transform = second_translate * r2;

    let cube1 = Cube::new_transform(Vector3::new(0., 0., 0.), Vector3::new(165., 165., 165.), 1, Arc::new(second_transform));
    vec.extend(cube1.get_sides());
    let cube2 = Cube::new_transform(Vector3::new(0., 0., 0.), Vector3::new(165., 330., 165.), 1, Arc::new(first_transform));
    vec.extend(cube2.get_sides());
    
    let len = vec.len();
    
    let node = BvhNode::new(&mut vec, 0, len, 0., 1.);
    (camera, node, vec, materials, textures)
}