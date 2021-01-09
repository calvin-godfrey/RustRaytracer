use nalgebra::geometry::Point2;
use crate::util;

pub struct Distribution1D {
    pub func: Vec<f64>, // function evaluated at `n` values
    cdf: Vec<f64>, // cdf of function
    pub func_int: f64 // integral of function
}

impl PartialEq for Distribution1D {
    fn eq(&self, other: &Self) -> bool {
        if self.func_int != other.func_int {
            return false;
        }
        if self.func.len() != other.func.len() {
            return false;
        }
        for i in 0usize..self.func.len() {
            if self.func[i] != other.func[i] {
                return false;
            }
        }
        true
    }
}

impl Distribution1D {
    pub fn make_distribution(f: &[f64]) -> Self {
        let n = f.len();
        let func = f.to_owned(); // make a copy of data
        let mut cdf = vec![0f64; n + 1];
        cdf[0] = 0f64;

        for i in 1..(n + 1) {
            // calculate cdf as the area of next rectangle
            cdf[i] = cdf[i - 1] + func[i - 1] / (n as f64);
        }
        let func_int = cdf[n];
        // normalize cdf
        if func_int == 0f64 {
            for i in 1..(n + 1) {
                cdf[i] = (i as f64) / (n as f64);
            }
        } else {
            for i in 1..(n + 1) {
                cdf[i] = cdf[i] / func_int;
            }
        }
        Self { func, cdf, func_int }
    }

    pub fn count(&self) -> usize { self.func.len() }

    /**
    * Returns value in [0, 1), pdf, and offset
    */
    pub fn sample_continuous(&self, u: f64) -> (f64, f64, usize) {
        let offset = find_interval(self.cdf.len(), &|index| self.cdf[index] <= u);
        // given a pair that straddle u, we can linearly interpolate to find x
        let mut du = u - self.cdf[offset];
        if self.cdf[offset + 1] - self.cdf[offset] > 0f64 {
            du = du / (self.cdf[offset + 1] - self.cdf[offset]);
        }
        let pdf = if self.func_int > 0f64 { self.func[offset] / self.func_int } else { 0f64 };
        let x = (offset as f64 + du) / (self.count() as f64);
        (x, pdf, offset)
    }

    /**
    * Returns offset, pdf, remapped u
    */
    pub fn sample_discrete(&self, u: f64) -> (usize, f64, f64) {
        let offset = find_interval(self.cdf.len(), &|index| self.cdf[index] <= u);
        let pdf = self.discrete_pdf(offset);
        let remapped = (u - self.cdf[offset]) / (self.cdf[offset + 1] - self.cdf[offset]);
        (offset, pdf, remapped)
    }

    pub fn discrete_pdf(&self, index: usize) -> f64 {
        self.func[index] / (self.func_int * self.count() as f64)
    }
}

pub struct Distribution2D {
    p_cond_v: Vec<Distribution1D>,
    p_marginal: Distribution1D,
}

impl PartialEq for Distribution2D {
    fn eq(&self, other: &Self) -> bool {
        if !self.p_marginal.eq(&other.p_marginal) {
            return false;
        }
        if self.p_cond_v.len() != other.p_cond_v.len() {
            return false;
        }
        for i in 0usize..self.p_cond_v.len() {
            if !self.p_cond_v[i].eq(&other.p_cond_v[i]) {
                return false;
            }
        }
        true
    }
}

impl Distribution2D {
    pub fn make_distribution_2d(f: &[f64], nu: usize, nv: usize) -> Self {
        let mut p_cond_v: Vec<Distribution1D> = Vec::new();
        for v in 0..nv {
            p_cond_v.push(Distribution1D::make_distribution(&f[v..(v + nu)]));
        }
        let mut marginal_func: Vec<f64> = Vec::new();
        for v in 0..nv {
            marginal_func.push(p_cond_v[v].func_int);
        }
        Self { p_cond_v, p_marginal: Distribution1D::make_distribution(&marginal_func) }
    }

    /**
    Returns sample, pdf
    */
    pub fn sample_continuous(&self, u: &Point2<f64>) -> (Point2<f64>, f64) {
        let (d1, pdf_1, v) = self.p_marginal.sample_continuous(u[1]);
        let (d0, pdf_0, ..) = self.p_cond_v[v].sample_continuous(u[0]);
        (Point2::new(d0, d1), pdf_1 * pdf_0)
    }

    pub fn pdf(&self, p: &Point2<f64>) -> f64 {
        let iu = (p[0] * self.p_cond_v[0].count() as f64) as usize;
        let iu = iu.max(0usize).min(self.p_cond_v[0].count() - 1);
        let iv = (p[1] * self.p_marginal.count() as f64) as usize;
        let iv = iv.max(0usize).min(self.p_marginal.count() - 1);
        self.p_cond_v[iv].func[iu] / self.p_marginal.func_int
    }
}

pub fn find_interval(size: usize, pred: &dyn Fn(usize) -> bool) -> usize {
    // binary search based on predicate
    let mut first: usize = 0;
    let mut len = size;
    while len > 0 {
        let half = len >> 1;
        let middle = first + half;
        if pred(middle) {
            first = middle + 1;
            len = len - half - 1;
        } else {
            len = half;
        }
    }
    // make sure it falls in the correcet range
    util::clamp((first - 1) as f64, 0f64, (size - 2) as f64) as usize
}