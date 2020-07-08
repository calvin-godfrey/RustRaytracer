use tobj;
use nalgebra::geometry::Point3;
use nalgebra::base::{Vector2, Vector3};
use crate::hittable;
use crate::materials;
use crate::consts::*;

pub fn parse_obj(path: &str, use_bb: bool) -> hittable::Mesh {
    let obj_res: Result<(Vec<tobj::Model>, Vec<tobj::Material>), tobj::LoadError> = tobj::load_obj(path, true);
    let mut ind: Vec<usize> = Vec::new();
    let mut p: Vec<Point3<f64>> = Vec::new();
    let mut n: Vec<Vector3<f64>> = Vec::new();
    let mut uv: Vec<Vector2<f64>> = Vec::new();
    let mut triangle_boxes: Vec<hittable::BoundingBox> = Vec::new();

    let mut min: Point3<f64> = Point3::new(INFINITY, INFINITY,  INFINITY);
    let mut max: Point3<f64> = Point3::new(-INFINITY, -INFINITY, -INFINITY);
    
    match obj_res {
        Ok((models, mats)) => {
            let model: &tobj::Model = &models[0];
            let mesh: &tobj::Mesh = &model.mesh;
            for (ind, pos) in mesh.positions.iter().enumerate() {
                if ind % 3 == 0 {
                    let x = *pos as f64;
                    let y = mesh.positions[ind + 1] as f64;
                    let z = mesh.positions[ind + 2] as f64;
                    min.x = min.x.min(x);
                    max.x = max.x.max(x);
                    min.y = min.y.min(y);
                    max.y = max.y.max(y);
                    min.z = min.z.min(z);
                    max.z = max.z.max(z);
                    p.push(Point3::new(x, y, z));
                }
            }
            let bbox = if use_bb { Some(hittable::BoundingBox::new(min, max)) } else { None };
            for (ind, pos) in mesh.normals.iter().enumerate() {
                if ind % 3 == 0 {
                    let x = *pos as f64;
                    let y = mesh.normals[ind + 1] as f64;
                    let z = mesh.normals[ind + 2] as f64;
                    n.push(Vector3::new(x, y, z));
                }
            }
            for (ind, pos) in mesh.texcoords.iter().enumerate() {
                if ind % 2 == 0 {
                    let u = *pos as f64;
                    let v = mesh.texcoords[ind + 1] as f64;
                    uv.push(Vector2::new(u, v));
                }
            }
            for (index, pos) in mesh.indices.iter().enumerate() {
                ind.push(*pos as usize);
                if use_bb && index % 3 == 0 {
                    let v1 = p[*pos as usize];
                    let v2 = p[mesh.indices[index + 1] as usize];
                    let v3 = p[mesh.indices[index + 2] as usize];
                    let x_min = v1.x.min(v2.x.min(v3.x));
                    let x_max = v1.x.max(v2.x.max(v3.x));
                    let y_min = v1.y.min(v2.y.min(v3.y));
                    let y_max = v1.y.max(v2.y.max(v3.y));
                    let z_min = v1.z.min(v2.y.min(v3.z));
                    let z_max = v1.z.max(v2.y.max(v3.z));
                    let tri_box = hittable::BoundingBox::new(Point3::new(x_min, y_min, z_min), Point3::new(x_max, y_max, z_max));
                    triangle_boxes.push(tri_box);
                }
            }
            if let Some(id) = mesh.material_id {
                let mesh_mat: &tobj::Material = &mats[id];
                let texture_path = &mesh_mat.ambient_texture[..];
                let real_mat: materials::Texture = materials::Texture::new(texture_path, Vector3::new(255., 0., 0.));
                println!("{}, {}", uv.len(), mesh.texcoords.len());
                return hittable::Mesh { ind, p, n, uv, mat: Box::new(real_mat), bounding_box: bbox, triangle_boxes };
            }
        },
        Err(_) => {
            panic!("Failed to parse obj {}", path);
        }
    }

    // random default, doesn't matter what it is
    hittable::Mesh { ind: Vec::new(), p: Vec::new(), n: Vec::new(), uv: Vec::new(), mat: Box::new(materials::Dielectric::new(1.)), bounding_box: None, triangle_boxes: Vec::new() }
}