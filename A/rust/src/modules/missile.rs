// Rust version of Missile.py
pub struct Missile {
    pub vx: f64,
    pub vy: f64,
    pub vz: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

const M: [[f64; 3]; 4] = [
    [0.0, 0.0, 0.0],
    [20000.0, 0.0, 2000.0],
    [19000.0, 600.0, 2100.0],
    [18000.0, -600.0, 1900.0],
];

impl Missile {
    pub fn new(x: f64, y: f64, z: f64, v: f64) -> Self {
        let norm = (x * x + y * y + z * z).sqrt();
        Missile {
            vx: v * x / norm,
            vy: v * y / norm,
            vz: v * z / norm,
            x,
            y,
            z,
        }
    }
    pub fn new_id(id: usize, v: f64) -> Self {
        let idx = if id < M.len() { id } else { 0 };
        let pos = M[idx];
        Missile::new(pos[0], pos[1], pos[2], v)
    }
    pub fn time_tick(&mut self, dt: f64) {
        self.x -= dt * self.vx;
        self.y -= dt * self.vy;
        self.z -= dt * self.vz;
    }
    pub fn time(&self, t: f64) -> Missile {
        Missile {
            vx: self.vx,
            vy: self.vy,
            vz: self.vz,
            x: self.x - t * self.vx,
            y: self.y - t * self.vy,
            z: self.z - t * self.vz,
        }
    }
}
