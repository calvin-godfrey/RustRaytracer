use tobj;
use nalgebra::geometry::{Projective3, Point3};
use nalgebra::base::{Vector2, Vector3};
use crate::hittable;
use crate::material::materials::{Texture};
use crate::consts::*;

#[allow(unused_variables)]
pub fn parse_obj(path: &str, trans: Projective3<f64>) -> hittable::Mesh {
    let obj_res: Result<(Vec<tobj::Model>, Vec<tobj::Material>), tobj::LoadError> = tobj::load_obj(path, true);
    let mut ind: Vec<usize> = Vec::new();
    let mut p: Vec<Point3<f64>> = Vec::new();
    let mut n: Vec<Vector3<f64>> = Vec::new();
    let mut uv: Vec<Vector2<f64>> = Vec::new();

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
                    let point = Point3::new(x, y, z);
                    let point: Point3<f64> = trans.transform_point(&point);
                    min.x = min.x.min(point.x);
                    max.x = max.x.max(point.x);
                    min.y = min.y.min(point.y);
                    max.y = max.y.max(point.y);
                    min.z = min.z.min(point.z);
                    max.z = max.z.max(point.z);
                    p.push(point);
                }
            }
            let bbox = Some(hittable::BoundingBox::new(min, max));
            for ind in (0..mesh.normals.len()).step_by(3) {
                let x = mesh.normals[ind] as f64;
                let y = mesh.normals[ind + 1] as f64;
                let z = mesh.normals[ind + 2] as f64;
                let vector = Vector3::new(x, y, z);
                let vector = trans.transform_vector(&vector);
                n.push(vector);
            }

            for (ind, pos) in mesh.texcoords.iter().enumerate() {
                if ind % 2 == 0 {
                    let u = *pos as f64;
                    let v = mesh.texcoords[ind + 1] as f64;
                    uv.push(Vector2::new(u, v));
                }
            }
            for pos in mesh.indices.iter() {
                ind.push(*pos as usize);
            }

            if let Some(id) = mesh.material_id {
                // let mesh_mat: &tobj::Material = &mats[id]; // TODO: use this
                // materials.push(mat);
                let new_mesh = hittable::Mesh { ind, p, n, uv, bounding_box: bbox };
                return new_mesh;
            } else {
                // materials.push(mat);
                let new_mesh = hittable::Mesh { ind, p, n, uv, bounding_box: bbox };
                return new_mesh;
            }
        },
        Err(_) => {
            panic!("Failed to parse obj {}", path);
        }
    }
}