use crate::util;
use nalgebra::geometry::Point3;
use nalgebra::base::{Vector3, Unit};

#[derive(Clone)]
pub struct Perlin {
    num_points: usize,
    perm_x: Vec<usize>,
    perm_y: Vec<usize>,
    perm_z: Vec<usize>,
    rand_vecs: Vec<Unit<Vector3<f64>>>,
    pub scale: f64,
}

impl Perlin {
    pub fn new(num_points: usize, scale: f64) -> Self {
        let mut rand_vecs: Vec<Unit<Vector3<f64>>> = Vec::with_capacity(num_points);
        for _ in 0..num_points {
            rand_vecs.push(Unit::new_normalize(util::rand_vector(-1., 1.)));
        }
        let perm_x = Perlin::generate_perm(num_points);
        let perm_y = Perlin::generate_perm(num_points);
        let perm_z = Perlin::generate_perm(num_points);
        Self { num_points, perm_x, perm_y, perm_z, rand_vecs, scale }
    }

    pub fn get_noise(&self, p: &Point3<f64>) -> f64 {
        let x = p.x * self.scale;
        let y = p.y * self.scale;
        let z = p.z * self.scale;
        let _i = x.floor() as usize;
        let _j = y.floor() as usize;
        let _k = z.floor() as usize;

        let mut u = x.fract();
        let mut v = y.fract();
        let mut w = z.fract();

        if u < 0. { u += 1. }
        if v < 0. { v += 1. }
        if w < 0. { w += 1. }

        let mut c = [Unit::new_normalize(Vector3::new(0., 1., 0.)); 8];
        for di in 0..2 {
            for dj in 0..2 {
                for dk in 0..2 {
                    let index = self.perm_x[(_i + di) & 255] ^ self.perm_y[(_j + dj) & 255] ^ self.perm_z[(_k + dk) & 255];
                    c[di + 2 * dj + 4 * dk] = self.rand_vecs[index];
                }
            }
        }

        u = u * u * (3. - 2. * u); // hermitan smoothing
        v = v * v * (3. - 2. * v);
        w = w * w * (3. - 2. * w);

        let mut accum: f64 = 0.;
        for i in 0..2 {
            for j in 0..2 {
                for k in 0..2 {
                    let weight = Vector3::new(u - i as f64, v - j as f64, w - k as f64);
                    let i_part = i as f64 * u + ((1-i) as f64) * (1.-u);
                    let j_part = j as f64 * v + ((1-j) as f64) * (1.-v);
                    let k_part = k as f64 * w + ((1-k) as f64) * (1.-w);
                    accum += i_part * j_part * k_part * c[i + 2 * j + 4 * k].dot(&weight);
                }
            }
        }
        accum
    }

    pub fn turb(&self, p: &Point3<f64>, depth: i32) -> f64 {
        let mut accum: f64 = 0.;
        let mut weight: f64 = 1.0;
        let mut point = p.clone();

        for _ in 0..depth {
            accum += weight * self.get_noise(&point);
            weight *= 0.5;
            point = Point3::new(point.x * 2., point.y * 2., point.z * 2.);
        }

        accum.abs()
    }

    fn generate_perm(num_points: usize) -> Vec<usize> {
        let mut p: Vec<usize> = vec![0; num_points];
        for i in 0..num_points {
            p[i] = i;
        }
        Perlin::permute(&mut p, num_points);
        p
    }

    fn permute(p: &mut Vec<usize>, num_points: usize) {
        for i in (0..num_points).rev() {
            let target = util::rand_int(0, i as i32) as usize;
            let temp = p[i];
            p[i] = p[target];
            p[target] = temp;
        }
    }
}