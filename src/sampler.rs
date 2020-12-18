#![allow(unused_variables, dead_code)]
use nalgebra::geometry::Point2;
use rand::prelude::*;
use crate::consts::*;

fn multiply_generator(c: &[u32], mut a: u32) -> u32 {
    let mut v: u32 = 0;
    let mut i: usize = 0;
    while a != 0 {
        if a & 1 > 0 {
            v ^= c[i];
        }
        i += 1;
        a >>= 1;
    }
    return v;
}

fn gray_code(n: u32) -> u32 {
    (n >> 1) ^ n
}

fn gray_code_sample1(c: &[u32], n: i64, scramble: u32, p: &mut[f64]) {
    let mut v = scramble;
    for i in 0i64..n {
        p[i as usize] = (v as f64 * 1f64 / (2i32.pow(32) as f64)).min(ONE_MINUS_EPSILON);
        v ^= c[(i + 1).trailing_zeros() as usize];
    }
}

fn gray_code_sample2(c1: &[u32], c2: &[u32], n: i64, scramble: Point2<u32>, p: &mut[Point2<f64>]) {
    let mut v = scramble;
    for i in 0i64..n {
        p[i as usize].x = (v.x as f64 * 1f64 / (2i32.pow(32) as f64)).min(ONE_MINUS_EPSILON);
        p[i as usize].y = (v.y as f64 * 1f64 / (2i32.pow(32) as f64)).min(ONE_MINUS_EPSILON);
        v.x ^= c1[(i + 1).trailing_zeros() as usize];
        v.y ^= c1[(i + 1).trailing_zeros() as usize];
    }
}

fn sample_generator_matrix(c: &[u32], a: u32, scramble: u32) -> f64 {
    (multiply_generator(c, a) ^ scramble) as f64 * 1f64 / (2i32.pow(32) as f64)
}

fn van_der_corput(n_samples_per_pixel_sample: i32, n_pixel_samples: i64, samples: &mut[f64]) {
    let mut rng = thread_rng();
    let scramble = rng.next_u32();
    let c_van_der_corput: Vec<u32> = vec![
        0x80000000, 0x40000000, 0x20000000, 0x10000000, 0x8000000, 0x4000000,
        0x2000000,  0x1000000,  0x800000,   0x400000,   0x200000,  0x100000,
        0x80000,    0x40000,    0x20000,    0x10000,    0x8000,    0x4000,
        0x2000,     0x1000,     0x800,      0x400,      0x200,     0x100,
        0x80,       0x40,       0x20,       0x10,       0x8,       0x4,
        0x2,        0x1
    ];
    let total: i64 = n_samples_per_pixel_sample as i64 * n_pixel_samples;
    gray_code_sample1(&c_van_der_corput[..], total, scramble, samples);
    for i in 0..n_pixel_samples {
        shuffle(&mut samples[(i * n_samples_per_pixel_sample as i64) as usize..], n_samples_per_pixel_sample, 1);
    }
    shuffle(samples, n_pixel_samples as i32, n_samples_per_pixel_sample);
}

fn sobol_2d(n_samples_per_pixel_sample: i32, n_pixel_samples: i64, samples: &mut[Point2<f64>]) {
    let mut rng = thread_rng();
    let scramble: Point2<u32> = Point2::new(rng.next_u32(), rng.next_u32());
    let c_sobol: Vec<Vec<u32>> = vec![vec![0x80000000, 0x40000000, 0x20000000, 0x10000000, 0x8000000, 0x4000000,
    0x2000000, 0x1000000, 0x800000, 0x400000, 0x200000, 0x100000, 0x80000,
    0x40000, 0x20000, 0x10000, 0x8000, 0x4000, 0x2000, 0x1000, 0x800,
    0x400, 0x200, 0x100, 0x80, 0x40, 0x20, 0x10, 0x8, 0x4, 0x2, 0x1],
    vec![0x80000000, 0xc0000000, 0xa0000000, 0xf0000000, 0x88000000, 0xcc000000,
    0xaa000000, 0xff000000, 0x80800000, 0xc0c00000, 0xa0a00000, 0xf0f00000,
    0x88880000, 0xcccc0000, 0xaaaa0000, 0xffff0000, 0x80008000, 0xc000c000,
    0xa000a000, 0xf000f000, 0x88008800, 0xcc00cc00, 0xaa00aa00, 0xff00ff00,
    0x80808080, 0xc0c0c0c0, 0xa0a0a0a0, 0xf0f0f0f0, 0x88888888, 0xcccccccc,
    0xaaaaaaaa, 0xffffffff]];

    gray_code_sample2(&c_sobol[0], &c_sobol[1], n_samples_per_pixel_sample as i64 * n_pixel_samples, scramble, samples);
    for i in 0..n_pixel_samples {
        shuffle(&mut samples[(i * n_samples_per_pixel_sample as i64) as usize..], n_samples_per_pixel_sample, 1);
    }
    shuffle(samples, n_pixel_samples as i32, n_samples_per_pixel_sample);
}

fn stratified_sample_1d(n_samples: i32, jitter: bool) -> Vec<f64> {
    let mut rng = thread_rng();
    let inv_samples = 1f64 / n_samples as f64;
    let mut v: Vec<f64> = Vec::with_capacity(n_samples as usize);
    for i in 0..n_samples {
        let delta = if jitter { rng.gen() } else { 0.5 };
        v[i as usize] = ONE_MINUS_EPSILON.min((i as f64 + delta) * inv_samples);
    }
    v
}

fn stratified_sample_2d(n_x: i32, n_y: i32, jitter: bool) -> Vec<Point2<f64>> {
    let mut rng = thread_rng();
    let dx = 1f64 / n_x as f64;
    let dy = 1f64 / n_y as f64;
    let mut v: Vec<Point2<f64>> = Vec::with_capacity((n_x * n_y) as usize);
    for y in 0..n_y {
        for x in 0..n_x {
            let jx = if jitter { rng.gen() } else { 0.5 };
            let jy = if jitter { rng.gen() } else { 0.5 };
            let p = Point2::new(ONE_MINUS_EPSILON.min((x as f64 + jx) * dx), (y as f64 + jy) * dy);
            v[((y * n_x) + x) as usize] = p;
        }
    }
    v
}

fn shuffle<T: Copy>(samples: &mut [T], count: i32, n_dimensions: i32) {
    let mut rng = thread_rng();
    for i in 0..count {
        let other = i + rng.gen_range(0., (count - i) as f64).floor() as i32;
        for j in 0..n_dimensions {
            let temp = samples[(n_dimensions * i + j) as usize];
            samples[(n_dimensions * i + j) as usize] = samples[(n_dimensions * other + j) as usize];
            samples[(n_dimensions * other + j) as usize] = temp;
        }
    }
}

fn latin_hyper_cube(samples: &mut[f64], n_samples: i64, n_dim: i32) {
    let mut rng = thread_rng();
    let inv_samples = 1f64 / n_samples as f64;
    // generate samples
    for i in 0..n_samples {
        for j in 0..n_dim {
            let sj = (i as f64 + (rng.gen_range(0f64, 1f64))) * inv_samples;
            samples[(n_dim as i64 * i + j as i64) as usize] = sj.min(ONE_MINUS_EPSILON);
        }
    }
    // permute
    for i in 0usize..n_dim as usize {
        for j in 0i64..n_samples {
            let other: u32 = j as u32 + rng.gen_range(0, (n_samples - j) as u32 + 1) as u32;
            let index1 = (n_dim as i64 * j + i as i64) as usize;
            let index2 = (n_dim as i64 * other as i64 + i as i64) as usize;
            let temp = samples[index1];
            samples[index1] = samples[index2];
            samples[index2] = temp;
        }
    }
}

#[derive(Clone)]
pub struct Sampler {
    pub samples_per_pixel: i64,
    current_pixel: Point2<i32>,
    current_pixel_index: i64,
    samples_1d_sizes: Vec<i32>,
    samples_2d_sizes: Vec<i32>,
    sample_array_1d: Vec<Vec<f64>>,
    sample_array_2d: Vec<Vec<Point2<f64>>>,
    array_1d_offset: usize,
    array_2d_offset: usize,
}

impl Sampler {
    pub fn new(samples: i64) -> Self {
        Self { samples_per_pixel: samples,
               current_pixel: Point2::new(0, 0),
               current_pixel_index: 0,
               samples_1d_sizes: Vec::new(),
               samples_2d_sizes: Vec::new(),
               sample_array_1d: Vec::new(),
               sample_array_2d: Vec::new(),
               array_1d_offset: 0,
               array_2d_offset: 0 }
    }

    pub fn start_pixel(&mut self, p: &Point2<i32>) {
        self.current_pixel = *p;
        self.current_pixel_index = 0;
        self.array_1d_offset = 0;
        self.array_2d_offset = 0;
    }

    pub fn start_next_sample(&mut self) -> bool {
        self.array_1d_offset = 0;
        self.array_2d_offset = 0;
        self.current_pixel_index += 1;
        return self.current_pixel_index < self.samples_per_pixel;
    }

    pub fn set_sample_number(&mut self, sample_num: i64) -> bool {
        self.array_1d_offset = 0;
        self.array_2d_offset = 0;
        self.current_pixel_index = sample_num;
        return self.current_pixel_index < self.samples_per_pixel;
    }

    pub fn request_1d_array(&mut self, n: i32) {
        self.samples_1d_sizes.push(n);
        self.sample_array_1d.push(Vec::with_capacity(((n as i64) * self.samples_per_pixel) as usize));
    }

    pub fn request_2d_array(&mut self, n: i32) {
        self.samples_2d_sizes.push(n);
        self.sample_array_2d.push(Vec::with_capacity(((n as i64) * self.samples_per_pixel) as usize));
    }

    pub fn get_1d_array(&mut self, n: i32) -> Option<(&Vec<f64>, usize)> {
        if self.array_1d_offset == self.sample_array_1d.len() {
            return None;
        }
        assert!(*self.samples_1d_sizes.get(self.array_1d_offset).unwrap() == n);
        let ans = (self.sample_array_1d.get(self.array_1d_offset).unwrap(), (self.current_pixel_index * (n as i64)) as usize);
        self.array_1d_offset += 1;
        Some(ans)
    }

    pub fn get_2d_array(&mut self, n: i32) -> Option<(&Vec<Point2<f64>>, usize)> {
        if self.array_2d_offset == self.sample_array_2d.len() {
            return None;
        }
        assert!(*self.samples_2d_sizes.get(self.array_2d_offset).unwrap() == n);
        let ans = (self.sample_array_2d.get(self.array_2d_offset).unwrap(), (self.current_pixel_index * (n as i64)) as usize);
        self.array_2d_offset += 1;
        Some(ans)
    }
}

#[derive(Clone)]
pub struct PixelSampler {
    samples_1d: Vec<Vec<f64>>,
    samples_2d: Vec<Vec<Point2<f64>>>,
    current_1d_dimension: i32,
    current_2d_dimension: i32,
    sampler: Sampler,
}

impl PixelSampler {
    pub fn new(samples: i64, n_dimensions: i32) -> Self {
        let mut samples_1d: Vec<Vec<f64>> = Vec::with_capacity(n_dimensions as usize);
        let mut samples_2d: Vec<Vec<Point2<f64>>> = Vec::with_capacity(n_dimensions as usize);
        for i in 0usize..n_dimensions as usize {
            samples_1d[i] = Vec::with_capacity(samples as usize);
            samples_2d[i] = Vec::with_capacity(samples as usize);
        }
        PixelSampler { samples_1d, samples_2d, current_1d_dimension: 0, current_2d_dimension: 0, sampler: Sampler::new(samples) }
    }

    pub fn start_pixel(&mut self, p: &Point2<i32>) {
        self.sampler.start_pixel(p);
    }

    pub fn start_next_sample(&mut self) -> bool {
        self.current_2d_dimension = 0;
        self.current_1d_dimension = 0;
        self.sampler.start_next_sample()
    }

    pub fn set_sample_number(&mut self, sample_num: i64) -> bool {
        self.current_2d_dimension = 0;
        self.current_1d_dimension = 0;
        self.sampler.set_sample_number(sample_num)
    }

    pub fn request_1d_array(&mut self, n: i32) {
        self.sampler.request_1d_array(n)
    }

    pub fn request_2d_array(&mut self, n: i32) {
        self.sampler.request_2d_array(n)
    }

    pub fn get_1d(&mut self) -> f64 {
        assert!(self.sampler.current_pixel_index < self.sampler.samples_per_pixel);
        if (self.current_1d_dimension as usize) < self.samples_1d.len() {
            let ans = self.samples_1d.get(self.current_1d_dimension as usize).unwrap().get(self.sampler.current_pixel_index as usize).unwrap();
            self.current_1d_dimension += 1;
            *ans
        } else {
            thread_rng().gen_range(0f64, 1f64)
        }
    }

    pub fn get_2d(&mut self) -> Point2<f64> {
        // println!("Current index: {}/{}", self.sampler.current_pixel_index, self.sampler.samples_per_pixel);
        assert!(self.sampler.current_pixel_index < self.sampler.samples_per_pixel);
        if (self.current_2d_dimension as usize) < self.samples_2d.len() {
            let ans = self.samples_2d.get(self.current_2d_dimension as usize).unwrap().get(self.sampler.current_pixel_index as usize).unwrap();
            self.current_2d_dimension += 1;
            *ans
        } else {
            Point2::new(thread_rng().gen_range(0f64, 1f64), thread_rng().gen_range(0f64, 1f64))
        }
    }
}

pub struct GlobalSampler {
    sampler: Sampler,
    dimension: i32,
    interval_sample_index: i64,
    array_end_dim: i32,
}

impl GlobalSampler {
    pub fn start_pixel(&mut self, p: &Point2<i32>, sampler: &Samplers) {
        self.sampler.start_pixel(p);
        self.dimension = 0;
        self.interval_sample_index = sampler.get_index_for_sample(0i64);
        self.array_end_dim = GlobalSampler::array_start_dim() + (self.sampler.sample_array_1d.len() + 2 * self.sampler.sample_array_2d.len()) as i32;
        
        for i in 0usize..self.sampler.samples_1d_sizes.len() {
            let n_samples = *self.sampler.samples_1d_sizes.get(i).unwrap() as i64 * self.sampler.samples_per_pixel;
            for j in 0..n_samples {
                let index: i64 = sampler.get_index_for_sample(j);
                self.sampler.sample_array_1d[i as usize][j as usize] = sampler.sample_dimension(index, GlobalSampler::array_start_dim() + i as i32);
            }
        }

        // same thing but 2d
        let mut dim = GlobalSampler::array_start_dim() + self.sampler.samples_1d_sizes.len() as i32;
        for i in 0usize..self.sampler.samples_2d_sizes.len() {
            let n_samples = *self.sampler.samples_2d_sizes.get(i).unwrap() as i64 * self.sampler.samples_per_pixel;
            for j in 0..n_samples {
                let index: i64 = sampler.get_index_for_sample(j);
                let x = sampler.sample_dimension(index, dim);
                let y = sampler.sample_dimension(index, dim + 1);
                self.sampler.sample_array_2d[i as usize][j as usize] = Point2::new(x, y);
            }
            dim += 2;
        }
        assert!(self.array_end_dim == dim);
    }

    pub fn start_next_sample(&mut self, sampler: &Samplers) -> bool {
        self.dimension = 0;
        self.interval_sample_index = sampler.get_index_for_sample(self.sampler.current_pixel_index + 1);
        self.sampler.start_next_sample()
    }

    pub fn set_sample_number(&mut self, sample_num: i64, sampler: &Samplers) -> bool {
        self.dimension = 0;
        self.interval_sample_index = sampler.get_index_for_sample(sample_num);
        self.sampler.set_sample_number(sample_num)
    }

    pub fn get_1d(&mut self, sampler: &Samplers) -> f64 {
        if self.dimension >= GlobalSampler::array_start_dim() && self.dimension < self.array_end_dim {
            self.dimension = self.array_end_dim;
        }
        let ans = sampler.sample_dimension(self.interval_sample_index, self.dimension);
        self.dimension += 1;
        ans
    }

    pub fn get_2d(&mut self, sampler: &Samplers) -> Point2<f64> {
        if self.dimension + 1 >= GlobalSampler::array_start_dim() && self.dimension < self.array_end_dim {
            self.dimension = self.array_end_dim;
        }
        let x = sampler.sample_dimension(self.interval_sample_index, self.dimension);
        let y = sampler.sample_dimension(self.interval_sample_index, self.dimension + 1);
        let ans = Point2::new(x, y);
        self.dimension += 2;
        ans
    }

    pub fn array_start_dim() -> i32 { 5 }
}

pub enum Samplers {
    StratifiedSampler {x_samples: i64, y_samples: i64, jitter_samples: bool, pixel: PixelSampler },
    ZeroTwoSequenceSampler { pixel: PixelSampler },
}

impl Clone for Samplers {
    fn clone(&self) -> Self {
        match self {
            Samplers::StratifiedSampler { x_samples, y_samples, jitter_samples, pixel } => { 
                Samplers::StratifiedSampler {x_samples: *x_samples, y_samples: *y_samples, jitter_samples: *jitter_samples, pixel: pixel.clone() }
             }
            Samplers::ZeroTwoSequenceSampler { pixel } => { Samplers::ZeroTwoSequenceSampler{ pixel: pixel.clone() } }
        }
    }
}


impl Samplers {
    pub fn round_count(&self, n: u32) -> u32 {
        match self {
            Samplers::StratifiedSampler { .. } => {  }
            Samplers::ZeroTwoSequenceSampler { pixel } => { return n.next_power_of_two(); }
        }
        n
    }
    pub fn start_pixel(s: &mut Samplers, p: &Point2<i32>) {
        match s {
            Samplers::StratifiedSampler { x_samples, y_samples, jitter_samples, pixel } => {
                pixel.start_pixel(p);
                for i in 0usize..pixel.samples_1d.len() {
                    pixel.samples_1d[i] = stratified_sample_1d((*x_samples * *y_samples) as i32, *jitter_samples);
                    shuffle(&mut pixel.samples_1d[i], (*x_samples * *y_samples) as i32, 1);
                }

                for i in 0usize..pixel.samples_2d.len() {
                    pixel.samples_2d[i] = stratified_sample_2d(*x_samples as i32, *y_samples as i32, *jitter_samples);
                    shuffle(&mut pixel.samples_2d[i], (*x_samples * *y_samples) as i32, 1);
                }

                for i in 0usize..pixel.sampler.samples_1d_sizes.len() {
                    for j in 0i64..pixel.sampler.samples_per_pixel {
                        let count: i64 = pixel.sampler.samples_2d_sizes[i] as i64;
                        let v = stratified_sample_1d(count as i32, *jitter_samples);
                        for k in (j * count as i64) as usize..((j + 1) * count) as usize {
                            pixel.sampler.sample_array_1d[i][k] = v[k - (j * count) as usize];
                        }
                        shuffle(&mut pixel.sampler.sample_array_1d[i][(j * count) as usize .. ((j + 1) * count) as usize], count as i32, 1);
                    }
                }
                for i in 0..pixel.sampler.samples_2d_sizes.len() {
                    for j in 0usize..pixel.sampler.samples_per_pixel as usize {
                        let count = pixel.sampler.samples_2d_sizes[i];
                        let samples = &mut pixel.sampler.sample_array_1d[i][j * (count as usize)..(j+1)*(count as usize)];
                        latin_hyper_cube(samples, count as i64, 2);
                    }
                }
            }
            Samplers::ZeroTwoSequenceSampler { pixel } => {
                pixel.start_pixel(p);
                for i in 0usize..pixel.samples_1d.len() {
                    van_der_corput(1, pixel.sampler.samples_per_pixel, &mut pixel.samples_1d[i][..]);
                }
                for i in 0usize..pixel.samples_2d.len() {
                    sobol_2d(1, pixel.sampler.samples_per_pixel, &mut pixel.samples_2d[i][..]);
                }
                // generate
                for i in 0usize..pixel.sampler.samples_1d_sizes.len() {
                    van_der_corput(pixel.sampler.samples_1d_sizes[i], pixel.sampler.samples_per_pixel, &mut pixel.sampler.sample_array_1d[i][..]);
                }
                for i in 0usize..pixel.sampler.samples_2d_sizes.len() {
                    sobol_2d(pixel.sampler.samples_2d_sizes[i], pixel.sampler.samples_per_pixel, &mut pixel.sampler.sample_array_2d[i][..]);
                }
            }
        }
    }

    pub fn start_next_sample(&mut self) -> bool {
        match self {
            Samplers::StratifiedSampler {pixel, .. } => {pixel.start_next_sample()}
            Samplers::ZeroTwoSequenceSampler { pixel } => {pixel.start_next_sample()}
        }
    }

    pub fn set_sample_number(&mut self, sample_num: i64) -> bool {
        match self {
            Samplers::StratifiedSampler {pixel, .. } => {pixel.set_sample_number(sample_num)}
            Samplers::ZeroTwoSequenceSampler { pixel } => {pixel.set_sample_number(sample_num)}
        }
    }

    pub fn request_1d_array(&mut self, n: i32) {
        match self {
            Samplers::StratifiedSampler {pixel, .. } => {pixel.request_1d_array(n)}
            Samplers::ZeroTwoSequenceSampler { pixel } => {pixel.request_1d_array(n)}
        }
    }

    pub fn request_2d_array(&mut self, n: i32) {
        match self {
            Samplers::StratifiedSampler {pixel, .. } => {pixel.request_2d_array(n)}
            Samplers::ZeroTwoSequenceSampler { pixel } => {pixel.request_2d_array(n)}
        }
    }

    pub fn get_1d(&mut self) -> f64 {
        match self {
            Samplers::StratifiedSampler {pixel, .. } => {pixel.get_1d()}
            Samplers::ZeroTwoSequenceSampler { pixel } => {pixel.get_1d()}
        }
    }

    pub fn get_2d(&mut self) -> Point2<f64> {
        match self {
            Samplers::StratifiedSampler {pixel, .. } => {pixel.get_2d()}
            Samplers::ZeroTwoSequenceSampler { pixel } => {pixel.get_2d()}
        }
    }

    /**
    Returns pFilm, time, pLens
    */
    pub fn get_camera_sample(&mut self, pixel: &Point2<i32>) -> (Point2<f64>, f64, Point2<f64>) {
        let pixels_f = Point2::new(pixel.x as f64, pixel.y as f64);
        let offset = self.get_2d();
        let p_film = Point2::new(pixels_f.x + offset.x, pixels_f.y + offset.y);
        let time = self.get_1d();
        let lens = self.get_2d();
        (p_film, time, lens)
    }

    pub fn sample_dimension(&self, index: i64, dim: i32) -> f64 {
        0.
    }

    pub fn get_index_for_sample(&self, sample: i64) -> i64 {
        0
    }

    pub fn new_stratified(jitter: bool, x_samples: i64, y_samples: i64, dimensions: i32) -> Self {
        let pixel = PixelSampler::new(x_samples * y_samples, dimensions);
        Samplers::StratifiedSampler {x_samples, y_samples, jitter_samples: jitter, pixel }
    }

    pub fn new_zero_two_sequence_sampler(samples_per_pixel: i64, n_sampled_dimensions: i32) -> Self {
        let pixel = PixelSampler::new((samples_per_pixel as u64).next_power_of_two() as i64, n_sampled_dimensions);
        Samplers::ZeroTwoSequenceSampler{ pixel }
    }
}

fn default_get_1d() -> f64 { 0. }
fn default_get_2d() -> Point2<f64> { Point2::new(0., 0.) }