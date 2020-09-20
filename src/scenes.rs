use nalgebra::base::{Matrix4, Vector3};
use nalgebra::geometry::{Point3, Rotation3, Projective3, Similarity3};
use std::sync::Arc;
use crate::hittable::*;
use crate::consts::*;
use crate::material::materials::{Material, Texture};
use crate::util;
use crate::geometry;
use crate::primitive::Primitive;

#[allow(dead_code)]
pub fn make_world() -> (geometry::Camera, BvhNode, Vec<Mesh>, Vec<Primitive>, Vec<usize>, Vec<Material>, Vec<Texture>, String) {
    assert!(AMBIENT_LIGHT == true);
    let from: Point3<f64> = Point3::new(13., 2.,3.);
    let to: Point3<f64> = Point3::new(0., 0., 0.0);
    let up: Vector3<f64> = Vector3::new(0., 1., 0.);

    let camera = geometry::Camera::new(from, to, up, ASPECT_RATIO, 20., 0.16, 10.);

    let mut world: Vec<Primitive> = Vec::new();
    let mut textures: Vec<Texture> = Vec::new();
    let mut materials: Vec<Material> = Vec::new();
    textures.push(Texture::new_solid_color(Vector3::new(201., 201., 201.).scale(1./256.)));
    textures.push(Texture::new_solid_color(Vector3::new(51., 75.5, 25.5).scale(1./256.)));
    textures.push(Texture::new_checkered(0, 1, 4.));
    materials.push(Material::make_matte(2, 0., 0));
    let ground = Primitive::new_sphere(Point3::new(0., -10000., 0.), 10000., 0);
    // let ground = Primitive::new_sphere(Point3::new(0., -10000., 0.), 10000., Arc::new(Material::new_lambertian(Texture::new_perlin(4.))));

    world.push(ground);

    for a in -11..11 {
        for b in -11..11 {
            let center = Point3::new(a as f64 + 0.9*util::rand(), 0.2, b as f64 + 0.9 * util::rand());
            if (center - Point3::new(4., 0.2, 0.)).norm() > 0.9 {
                let mat = util::rand();
                if mat < 0.6 { // matte
                    let color: Vector3<f64> = Vector3::new(255. * util::rand() * util::rand_range(0.4, 0.8), 255. * util::rand() * util::rand_range(0.4, 0.8), 255. * util::rand() * util::rand_range(0.4, 0.8)).scale(1./256.);
                    let texture = Texture::new_solid_color(color);
                    textures.push(texture);
                    let mat = Material::make_matte(textures.len() - 1, 0., 0);
                    materials.push(mat);
                    world.push(Primitive::new_sphere(center, 0.2, materials.len() - 1));
                } else if mat < 0.86 { // metal
                    let color: Vector3<f64> = Vector3::new(util::rand_range(30., 200.), util::rand_range(30., 200.), util::rand_range(30., 200.)).scale(1./256.);
                    textures.push(Texture::new_solid_color(color));
                    let mat = Material::make_mirror(textures.len() - 1, 0);
                    materials.push(mat);
                    world.push(Primitive::new_sphere(center, 0.2, materials.len() - 1));
                } else { // glass
                    let r = util::rand_range(0.5, 1.);
                    let g = util::rand_range(0.5, 1.);
                    let b = util::rand_range(0.5, 1.);
                    textures.push(Texture::new_solid_color(Vector3::new(r, g, b)));
                    let mat = Material::make_glass(textures.len() - 1, textures.len() - 1, 0., 0., util::rand_range(1.2, 1.8), 0, true);
                    materials.push(mat);
                    world.push(Primitive::new_sphere(center, 0.2, materials.len() - 1));
                }
            }
        }
    }

    let new_texture = Texture::new_perlin(6.);
    textures.push(new_texture);
    let perlin_mat = Material::make_matte(textures.len() - 1, 45., 0);
    materials.push(perlin_mat);
    world.push(Primitive::new_sphere(Point3::new(-4., 1., 0.), 1., materials.len() - 1));

    textures.push(Texture::new_solid_color(util::white()));
    materials.push(Material::make_glass(textures.len() - 1, textures.len() - 1, 0., 0., 1.3, 0, true));
    world.push(Primitive::new_sphere(Point3::new(0., 1., 0.), 1., materials.len() - 1));

    textures.push(Texture::new_solid_color(Vector3::new(223.5, 85., 32.5).scale(1./256.)));
    textures.push(Texture::new_solid_color(util::black()));
    textures.push(Texture::new_solid_color(Vector3::new(0.1, 0., 0.)));
    materials.push(Material::make_metal(textures.len() - 3, textures.len() - 2, textures.len() - 1, textures.len() - 1, textures.len() - 1, 0, true));
    world.push(Primitive::new_sphere(Point3::new(4., 1., 0.), 1., materials.len() - 1));

    let len = world.len();
    let mut indices: Vec<usize> = (0usize..len).collect();
    let node = BvhNode::new(&world, &mut indices, 0, len, 0., 0.);

    let lights = vec![];
    (camera, node, vec![], world, lights, materials, textures, "sphere_world.png".to_string())
}

#[allow(dead_code)]
pub fn cornell_box() -> (geometry::Camera, BvhNode, Vec<Mesh>, Vec<Primitive>, Vec<usize>, Vec<Material>, Vec<Texture>, String) {

    let from = Point3::new(278., 278., -800.);
    let to = Point3::new(278., 278., 0.);
    let up = Vector3::new(0., 1., 0.);

    let camera = geometry::Camera::new_motion_blur(from, to, up, ASPECT_RATIO, 40., 0., 10., 0., 1.);

    let mut vec: Vec<Primitive> = Vec::new();
    let mut materials: Vec<Material> = Vec::new();
    let mut textures: Vec<Texture> = Vec::new();
    textures.push(Texture::new_solid_color(Vector3::new(0.65, 0.05, 0.05))); // red
    textures.push(Texture::new_solid_color(Vector3::new(0.73, 0.73, 0.73))); // white
    textures.push(Texture::new_solid_color(Vector3::new(0.12, 0.45, 0.15))); // green
    textures.push(Texture::new_solid_color(Vector3::new(7., 7., 7.).scale(4.))); // light

    materials.push(Material::make_matte(0, 0., 0));
    materials.push(Material::make_matte(1, 0., 0));
    materials.push(Material::make_matte(2, 0., 0));
    materials.push(Material::make_light(3));
    
    vec.push(Primitive::new_flip_face(Box::new(Primitive::new_yz_rect(0., 0., 555., 555., 555., 2))));
    vec.push(Primitive::new_yz_rect(0., 0., 555., 555., 0., 0));
    vec.push(Primitive::new_flip_face(Box::new(Primitive::new_xz_rect(213., 227., 343., 332., 554.9, 3))));
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
    let mut indices: Vec<usize> = (0usize..len).collect();

    let lights = vec![2usize];
    
    let node = BvhNode::new(&vec, &mut indices, 0, len, 0., 1.);
    (camera, node, vec![], vec, lights, materials, textures, "cornell_box.png".to_string())
}

#[allow(dead_code)]
pub fn with_everything() -> (geometry::Camera, BvhNode, Vec<Mesh>, Vec<Primitive>, Vec<usize>, Vec<Material>, Vec<Texture>, String) {
    let from = Point3::new(478., 278., -600.);
    let to = Point3::new(278., 278., 0.);
    let up = Vector3::new(0., 1., 0.);

    let camera = geometry::Camera::new_motion_blur(from, to, up, ASPECT_RATIO, 40., 0., 10., 0., 1.);
    let mut textures: Vec<Texture> = Vec::new();
    let mut materials: Vec<Material> = Vec::new();
    let mut objs: Vec<Primitive> = Vec::new();
    textures.push(Texture::new_solid_color(Vector3::new(122.4, 211.7, 135.3).scale(1./256.))); // ground
    materials.push(Material::make_matte(0, 0., 0));
    let boxes_per_side = 20;
    for i in 0..boxes_per_side {
        for j in 0..boxes_per_side {
            let w = 100.;
            let x0 = -1000.0 + i as f64 * w;
            let z0 = -1000.0 + j as f64 * w;
            let y0 = 0.0;
            let x1 = x0 + w;
            let z1 = z0 + w;
            let y1 = util::rand_range(1., 91.);
            let cube = Cube::new(Vector3::new(x0, y0, z0), Vector3::new(x1, y1, z1), 0);
            objs.extend(cube.get_sides());
        }
    }

    textures.push(Texture::new_solid_color(Vector3::new(7., 7., 7.))); // light
    materials.push(Material::make_light(textures.len() - 1));
    objs.push(Primitive::new_xz_rect(123., 147., 423., 412., 554., materials.len() - 1));
    let lights = vec![objs.len() - 1];

    textures.push(Texture::new_solid_color(Vector3::new(178.5, 76.5, 25.5).scale(1./256.))); // moving sphere
    materials.push(Material::make_matte(0, 0., 0));
    objs.push(Primitive::new_moving_sphere(Point3::new(400., 400., 200.), Point3::new(430., 400., 200.),0., 1., 50., materials.len() - 1));

    textures.push(Texture::new_solid_color(util::white()));
    materials.push(Material::make_glass(textures.len() - 1, textures.len() - 1, 0., 0., 1.5, 0, true));
    objs.push(Primitive::new_sphere(Point3::new(260., 150., 45.), 50., materials.len() - 1)); // dielectric

    // metal
    textures.push(Texture::new_solid_color(util::white() - Vector3::new(204./256., 204./256., 229.3/256.)));
    textures.push(Texture::new_solid_color(util::black()));
    textures.push(Texture::new_solid_color(Vector3::new(0.1, 0., 0.)));
    materials.push(Material::make_metal(textures.len() - 3, textures.len() - 2, textures.len() - 1, textures.len() - 1, textures.len() - 1, 0, true));
    objs.push(Primitive::new_sphere(Point3::new(0., 150., 145.), 50., materials.len() - 1));

     // TODO: SubstrateMaterial
    // objs.push(Primitive::new_sphere(Point3::new(360., 150., 145.), 70., materials.len() - 2));
    // textures.push(Texture::new_solid_color(Vector3::new(81., 132., 249.6)));
    // materials.push(Material::new_lambertian(textures.len() - 1));
    // objs.push(Primitive::new_sphere(Point3::new(360., 150., 145.), 70., 3));
    
    textures.push(Texture::new_texture("data/earthmap.jpg")); // textured sphere
    materials.push(Material::make_matte(textures.len() - 1, 0., 0));
    objs.push(Primitive::new_sphere(Point3::new(400., 200., 400.), 100., materials.len() - 1));

    textures.push(Texture::new_solid_color(Vector3::new(186.2, 186.2, 186.2).scale(1./256.)));
    materials.push(Material::make_matte(textures.len() - 1, 0., 0));
    let rotate = Rotation3::from_euler_angles(60. * PI / 180., 34. * PI / 180., 0.);
    let translate = Projective3::from_matrix_unchecked(Matrix4::identity().append_translation(&Vector3::new(-17., 353., 478.)));
    let transform: Projective3<f64> = translate * rotate;
    for _ in 0..1000 {
        let x = util::rand_range(-83., 83.);
        let y = util::rand_range(-83., 83.);
        let z = util::rand_range(-83., 83.);
        let center = Point3::new(x, y, z);
        let new_center = transform.transform_point(&center);
        objs.push(Primitive::new_sphere(new_center, 10., materials.len() - 1));
    }

    let len = objs.len();
    let mut indices: Vec<usize> = (0usize..len).collect();
    (camera, BvhNode::new(&objs, &mut indices, 0, len, 0., 1.), vec![], objs, lights, materials, textures, "old_everything.png".to_string())
}

#[allow(dead_code)]
pub fn dragon(i: i32) -> (geometry::Camera, BvhNode, Vec<Mesh>, Vec<Primitive>, Vec<usize>, Vec<Material>, Vec<Texture>, String) {
    let mut meshes = vec![];
    let sand = Vector3::new(0.761, 0.698, 0.502).scale(1. / 0.761);
    let transform = Similarity3::new(Vector3::new(0., 0., 0.), Vector3::new(0., 0., 0.), 10.);
    let mesh = Mesh::new("data/dragon/dragon.obj", Projective3::from_matrix_unchecked(transform.to_homogeneous()), 1);
    meshes.push(mesh);
    let radians = (i as f64) * PI / 180.;
    let path = format!("dragon_{}.png", i);
    let r = 82.26f64.sqrt();
    let from: Point3<f64> = Point3::new(r * (radians + PI / 4.4).sin(), 4., r * (radians + PI / 4.4).cos());
    let to: Point3<f64> = Point3::new(0., -0.15, -0.08);
    let up: Vector3<f64> = Vector3::new(0., 1., 0.);

    let camera = geometry::Camera::new(from, to, up, ASPECT_RATIO, 70., 0.0, 10.);

    let mut world: Vec<Primitive> = Vec::new();
    let mut texts: Vec<Texture> = Vec::new();
    let mut mats: Vec<Material> = Vec::new();

    let light_gray = Vector3::new(0.4, 0.15, 0.15).scale(2.);
    let dark_gray = Vector3::new(0.15, 0.15, 0.4).scale(2.);
    let temp_len = texts.len();
    texts.push(Texture::new_solid_color(light_gray));
    texts.push(Texture::new_solid_color(dark_gray));
    texts.push(Texture::new_checkered(temp_len, temp_len+1, 0.1));
    texts.push(Texture::new_solid_color(sand));
    mats.push(Material::make_matte(2, 0f64, 0));
    mats.push(Material::make_matte(3, 45., 0));
    world.push(Primitive::new_xz_rect(-10000., -10000., 10000., 10000., -2.83, 0));

    let triangles = Mesh::generate_triangles(&meshes, 0, 1);
    for tri in triangles {
        world.push(tri);
    }
    texts.push(Texture::new_solid_color(Vector3::new(10., 10., 10.)));
    mats.push(Material::make_light(4));
    world.push(Primitive::new_xz_rect(-5., -5., 5., 5., 15., 2));

    let len = world.len();
    let mut indices: Vec<usize> = (0usize..len).collect();
    let bvh = BvhNode::new(&world, &mut indices, 0, len, 0., 1.);
    (camera, bvh, meshes, world, vec![], mats, texts, path)
}

#[allow(dead_code)]
pub fn plastic_dragon(i: i32) -> (geometry::Camera, BvhNode, Vec<Mesh>, Vec<Primitive>, Vec<usize>, Vec<Material>, Vec<Texture>, String) {
    let mut meshes = vec![];
    let transform = Similarity3::new(Vector3::new(0., 0., 0.), Vector3::new(0., 0., 0.), 10.);
    let mesh = Mesh::new("data/dragon/dragon.obj", Projective3::from_matrix_unchecked(transform.to_homogeneous()), 1);
    meshes.push(mesh);
    let radians = (i as f64) * PI / 180.;
    let path = format!("dragon_{}.png", i);
    let r = 82.26f64.sqrt();
    let from: Point3<f64> = Point3::new(r * (radians + PI / 4.4).sin(), 4., r * (radians + PI / 4.4).cos());
    // let to: Point3<f64> = Point3::new(0., -0.15, -0.08);
    let to: Point3<f64> = Point3::new(0.,1.1, 4.4);
    let up: Vector3<f64> = Vector3::new(0., 1., 0.);

    let camera = geometry::Camera::new(from, to, up, ASPECT_RATIO, 15., 0.0, 10.);

    let mut world: Vec<Primitive> = Vec::new();
    let mut texts: Vec<Texture> = Vec::new();
    let mut mats: Vec<Material> = Vec::new();

    let light_gray = Vector3::new(0.4, 0.15, 0.15).scale(2.);
    let dark_gray = Vector3::new(0.15, 0.15, 0.4).scale(2.);
    let temp_len = texts.len();
    texts.push(Texture::new_solid_color(light_gray));
    texts.push(Texture::new_solid_color(dark_gray));
    texts.push(Texture::new_checkered(temp_len, temp_len+1, 0.1));
    let purple = Vector3::new(0.1514, 0.0139, 0.3765).scale(0.4 / 0.3765);
    let spec_color = util::white().scale(1.);
    texts.push(Texture::new_solid_color(purple));
    texts.push(Texture::new_solid_color(spec_color));
    mats.push(Material::make_matte(2, 0f64, 0));
    mats.push(Material::make_plastic(3, 4, 0, 0.0, true));
    world.push(Primitive::new_xz_rect(-10000., -10000., 10000., 10000., -2.83, 0));

    let triangles = Mesh::generate_triangles(&meshes, 0, 1);
    for tri in triangles {
        world.push(tri);
    }
    texts.push(Texture::new_solid_color(Vector3::new(10., 10., 10.)));
    mats.push(Material::make_light(5));
    world.push(Primitive::new_xz_rect(-5., -5., 5., 5., 15., 2));

    let len = world.len();
    let mut indices: Vec<usize> = (0usize..len).collect();
    let bvh = BvhNode::new(&world, &mut indices, 0, len, 0., 1.);
    (camera, bvh, meshes, world, vec![], mats, texts, path)
}

#[allow(dead_code)]
pub fn glass_dragon(i: i32, urough: f64, vrough: f64, index: f64) -> (geometry::Camera, BvhNode, Vec<Mesh>, Vec<Primitive>, Vec<usize>, Vec<Material>, Vec<Texture>, String) {
    let mut meshes = vec![];
    let transform = Similarity3::new(Vector3::new(0., 0., 0.), Vector3::new(0., 0., 0.), 10.);
    let mesh = Mesh::new("data/dragon/dragon.obj", Projective3::from_matrix_unchecked(transform.to_homogeneous()), 1);
    meshes.push(mesh);
    let radians = (i as f64) * PI / 180.;
    let path = format!("dragon_{}.png", i);
    let r = 82.26f64.sqrt();
    let from: Point3<f64> = Point3::new(r * (radians + PI / 4.4).sin(), 4., r * (radians + PI / 4.4).cos());
    let to: Point3<f64> = Point3::new(0., -0.15, -0.08);
    let up: Vector3<f64> = Vector3::new(0., 1., 0.);

    let camera = geometry::Camera::new(from, to, up, ASPECT_RATIO, 70., 0.0, 10.);

    let mut world: Vec<Primitive> = Vec::new();
    let mut texts: Vec<Texture> = Vec::new();
    let mut mats: Vec<Material> = Vec::new();

    let light_gray = Vector3::new(0.4, 0.15, 0.15).scale(2.);
    let dark_gray = Vector3::new(0.15, 0.15, 0.4).scale(2.);
    let temp_len = texts.len();
    texts.push(Texture::new_solid_color(light_gray));
    texts.push(Texture::new_solid_color(dark_gray));
    texts.push(Texture::new_checkered(temp_len, temp_len+1, 0.1));
    let k_d = util::white();
    let k_t = util::white();
    texts.push(Texture::new_solid_color(k_d));
    texts.push(Texture::new_solid_color(k_t));
    mats.push(Material::make_matte(2, 0f64, 0));
    mats.push(Material::make_glass(3, 4, urough, vrough, index, 0, true));
    world.push(Primitive::new_xz_rect(-10000., -10000., 10000., 10000., -2.83, 0));

    let triangles = Mesh::generate_triangles(&meshes, 0, 1);
    for tri in triangles {
        world.push(tri);
    }
    texts.push(Texture::new_solid_color(Vector3::new(10., 10., 10.)));
    mats.push(Material::make_light(5));

    let len = world.len();
    let mut indices: Vec<usize> = (0usize..len).collect();
    let bvh = BvhNode::new(&world, &mut indices, 0, len, 0., 1.);
    (camera, bvh, meshes, world, vec![], mats, texts, path)
}

#[allow(dead_code)]
pub fn metal_dragon(i: i32, eta: Vector3<f64>, k: Vector3<f64>, ur: f64, vr: f64, ro: f64) -> (geometry::Camera, BvhNode, Vec<Mesh>, Vec<Primitive>, Vec<usize>, Vec<Material>, Vec<Texture>, String) {
    let mut meshes = vec![];
    let transform = Similarity3::new(Vector3::new(0., 0., 0.), Vector3::new(0., 0., 0.), 10.);
    let mesh = Mesh::new("data/dragon/dragon.obj", Projective3::from_matrix_unchecked(transform.to_homogeneous()), 1);
    meshes.push(mesh);
    let radians = (i as f64) * PI / 180.;
    let path = format!("dragon_{}.png", i);
    let r = 82.26f64.sqrt();
    let from: Point3<f64> = Point3::new(r * (radians + PI / 4.4).sin(), 4., r * (radians + PI / 4.4).cos());
    let to: Point3<f64> = Point3::new(0., -0.15, -0.08);
    let up: Vector3<f64> = Vector3::new(0., 1., 0.);

    let camera = geometry::Camera::new(from, to, up, ASPECT_RATIO, 70., 0.0, 10.);

    let mut world: Vec<Primitive> = Vec::new();
    let mut texts: Vec<Texture> = Vec::new();
    let mut mats: Vec<Material> = Vec::new();

    let light_gray = Vector3::new(0.4, 0.15, 0.15).scale(2.);
    let dark_gray = Vector3::new(0.15, 0.15, 0.4).scale(2.);
    let temp_len = texts.len();
    texts.push(Texture::new_solid_color(light_gray));
    texts.push(Texture::new_solid_color(dark_gray));
    texts.push(Texture::new_checkered(temp_len, temp_len+1, 0.1));
    let k_d = util::white();
    let k_t = util::white();
    texts.push(Texture::new_solid_color(k_d)); // 3
    texts.push(Texture::new_solid_color(k_t)); // 4
    texts.push(Texture::new_solid_color(Vector3::new(ur, 0., 0.))); // 5
    texts.push(Texture::new_solid_color(Vector3::new(vr, 0., 0.))); // 6
    texts.push(Texture::new_solid_color(Vector3::new(ro, 0., 0.))); // 7
    texts.push(Texture::new_solid_color(eta)); // 8
    texts.push(Texture::new_solid_color(k)); // 9
    mats.push(Material::make_matte(2, 0f64, 0));
    mats.push(Material::make_metal(8, 9, 5, 6, 7, 0, true));
    world.push(Primitive::new_xz_rect(-10000., -10000., 10000., 10000., -2.83, 0));

    let triangles = Mesh::generate_triangles(&meshes, 0, 1);
    for tri in triangles {
        world.push(tri);
    }

    let len = world.len();
    let mut indices: Vec<usize> = (0usize..len).collect();
    let bvh = BvhNode::new(&world, &mut indices, 0, len, 0., 1.);
    (camera, bvh, meshes, world, vec![], mats, texts, path)
}

#[allow(dead_code)]
pub fn sphere_roughness() -> (geometry::Camera, BvhNode, Vec<Mesh>, Vec<Primitive>, Vec<usize>, Vec<Material>, Vec<Texture>, String) {
    let meshes = vec![];
    let from: Point3<f64> = Point3::new(-8.5, 5., 0.);
    let to: Point3<f64> = Point3::new(0., -0.15, -0.08);
    let up: Vector3<f64> = Vector3::new(0., 1., 0.);

    let camera = geometry::Camera::new(from, to, up, ASPECT_RATIO, 70., 0.0, 10.);

    let mut world: Vec<Primitive> = Vec::new();
    let mut texts: Vec<Texture> = Vec::new();
    let mut mats: Vec<Material> = Vec::new();

    let light_gray = Vector3::new(0.4, 0.15, 0.15).scale(2.);
    let dark_gray = Vector3::new(0.15, 0.15, 0.4).scale(2.);
    let temp_len = texts.len();
    texts.push(Texture::new_solid_color(light_gray));
    texts.push(Texture::new_solid_color(dark_gray));
    texts.push(Texture::new_checkered(temp_len, temp_len+1, 0.1));
    texts.push(Texture::new_solid_color(Vector3::new(15., 15., 15.)));
    let eta = Vector3::new(0.05, 0.5, 0.75);
    let k = Vector3::new(0., 0., 0.);
    mats.push(Material::make_matte(2, 0f64, 0));
    let space: f64 = 2.8;

    for i in 1..9 {
        let len = texts.len();
        texts.push(Texture::new_solid_color(Vector3::new(i as f64 / 10. + SMALL, 0., 0.))); // u roughness
        texts.push(Texture::new_solid_color(Vector3::new(i as f64 / 10. + SMALL, 0., 0.))); // v roughness
        texts.push(Texture::new_solid_color(Vector3::new(i as f64 / 10. + SMALL, 0., 0.))); // roughness
        texts.push(Texture::new_solid_color(eta));
        texts.push(Texture::new_solid_color(k));
        mats.push(Material::make_metal(len + 3, len + 4, len, len + 1, len + 2, 0, true));
        world.push(Primitive::new_sphere(Point3::new(0., 1., -space * 4.5 + space * i as f64), 1., i));
    }
    world.push(Primitive::new_xz_rect(-10000., -10000., 10000., 10000., -0.01, 0));

    mats.push(Material::make_light(1));
    world.push(Primitive::new_xz_rect(-10., -10., 10., 10., 50., mats.len() - 1));

    let len = world.len();
    let mut indices: Vec<usize> = (0usize..len).collect();
    let bvh = BvhNode::new(&world, &mut indices, 0, len, 0., 1.);
    (camera, bvh, meshes, world, vec![], mats, texts, "metal_spheres.png".to_string())
}

#[allow(dead_code)]
pub fn two_dragons() -> (geometry::Camera, BvhNode, Vec<Mesh>, Vec<Primitive>, Vec<usize>, Vec<Material>, Vec<Texture>, String) {
    let mut meshes = vec![];
    let transform = Similarity3::new(Vector3::new(0., 0., 0.), Vector3::new(0., 0., 0.), 10.);
    let other_transform = Similarity3::new(Vector3::new(5., 0., 0.), Vector3::new(0., 0., 0.), 10.);
    let mesh = Mesh::new("data/dragon/dragon.obj", Projective3::from_matrix_unchecked(transform.to_homogeneous()), 2);
    meshes.push(mesh);
    meshes.push(Mesh::new("data/dragon/dragon.obj", Projective3::from_matrix_unchecked(other_transform.to_homogeneous()), 3));
    let from: Point3<f64> = Point3::new(-8.5, 5., 0.);
    let to: Point3<f64> = Point3::new(0., -0.15, -0.08);
    let up: Vector3<f64> = Vector3::new(0., 1., 0.);

    let camera = geometry::Camera::new(from, to, up, ASPECT_RATIO, 70., 0.0, 10.);

    let mut world: Vec<Primitive> = Vec::new();
    let mut texts: Vec<Texture> = Vec::new();
    let mut mats: Vec<Material> = Vec::new();

    let light_gray = Vector3::new(0.4, 0.15, 0.15).scale(2.);
    let dark_gray = Vector3::new(0.15, 0.15, 0.4).scale(2.);
    let temp_len = texts.len();
    texts.push(Texture::new_solid_color(light_gray));
    texts.push(Texture::new_solid_color(dark_gray));
    texts.push(Texture::new_checkered(temp_len, temp_len+1, 0.1));
    texts.push(Texture::new_solid_color(Vector3::new(15., 15., 15.))); // 4
    let eta = Vector3::new(0.05, 0.5, 0.75);
    let k = Vector3::new(0., 0., 0.);
    mats.push(Material::make_matte(2, 0f64, 0));    
    world.push(Primitive::new_xz_rect(-10000., -10000., 10000., 10000., -2.83, 0));
    mats.push(Material::make_light(3));
    world.push(Primitive::new_xz_rect(-10., -10., 10., 10., 50., mats.len() - 1));
    texts.push(Texture::new_solid_color(util::white())); // 4
    texts.push(Texture::new_solid_color(util::white())); // 5
    texts.push(Texture::new_solid_color(eta)); // 6
    texts.push(Texture::new_solid_color(k)); // 7
    texts.push(Texture::new_solid_color(Vector3::new(0.1, 0., 0.))); // 8
    mats.push(Material::make_glass(4, 5, 0.0, 0.0, 1.5, 0, true));
    mats.push(Material::make_metal(6, 7, 8, 8, 8, 0, true));

    let triangles = Mesh::generate_triangles(&meshes, 0, 2);
    for tri in triangles {
        world.push(tri);
    }

    let triangles = Mesh::generate_triangles(&meshes, 1, 3);
    for tri in triangles {
        world.push(tri);
    }

    let len = world.len();
    let mut indices: Vec<usize> = (0usize..len).collect();
    let bvh = BvhNode::new(&world, &mut indices, 0, len, 0., 1.);
    (camera, bvh, meshes, world, vec![], mats, texts, "two_dragons.png".to_string())
}

