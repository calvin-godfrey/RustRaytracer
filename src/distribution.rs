use crate::util;

pub struct Distribution1D {
    func: Vec<f64>, // function evaluated at `n` values
    cdf: Vec<f64>, // cdf of function
    func_int: f64 // integral of function
}

impl Distribution1D {
    pub fn make_distribution(f: &[f64]) -> Self {
        let n = f.len();
        let func = f.to_owned(); // make a copy of data
        let mut cdf = Vec::with_capacity(n+ 1);
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
                cdf[i] = cdf[i] / (n as f64);
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
        let pdf = self.func[offset] / self.func_int;
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