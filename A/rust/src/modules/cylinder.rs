// Rust version of Cylinder.py
use rand::Rng;
pub struct Cylinder {
    pub r: f64,
    pub height: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Cylinder {
    pub fn new(r: f64, height: f64, x: f64, y: f64, z: f64) -> Self {
        Cylinder { r, height, x, y, z }
    }
    pub fn random_point_on_surface(&self) -> (f64, f64, f64) {
        let mut rng = rand::thread_rng();
        let face: f64 = rng.gen_range(0.0..1.0);
        let pi = std::f64::consts::PI;
        let area_top_bottom = pi * self.r * self.r;
        let area_side = pi * self.r * self.height;
        let prob_face = area_top_bottom / (area_side + area_top_bottom);

        if face < prob_face {
            // top or bottom face
            let theta: f64 = rng.gen_range(0.0..2.0 * pi);
            let val: f64 = rng.gen_range(0.0..1.0);
            let r0 = self.r * val.sqrt();
            let x = self.x + r0 * theta.cos();
            let y = self.y + r0 * theta.sin();
            let z = self.z + if rng.gen_bool(0.5) { 0.0 } else { self.height };
            (x, y, z)
        } else {
            // curved surface
            let theta: f64 = rng.gen_range(0.0..2.0 * pi);
            let z = self.z + rng.gen_range(0.0..self.height);
            let x = self.x + self.r * theta.cos();
            let y = self.y + self.r * theta.sin();
            (x, y, z)
        }
    }
}
