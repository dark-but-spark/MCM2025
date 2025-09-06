// Rust version of SmokeBall.py

pub struct SmokeBall {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub r: f64,
    pub v: f64,
}

impl SmokeBall {
    pub fn new(x: f64, y: f64, z: f64, r: f64, v: f64) -> Self {
        SmokeBall { x, y, z, r, v }
    }
    pub fn time_tick(&mut self, dt: f64) {
        self.z += self.v * dt;
    }
    pub fn time(&self, t: f64) -> SmokeBall {
        SmokeBall {
            x: self.x,
            y: self.y,
            z: self.z + self.v * t,
            r: self.r,
            v: self.v,
        }
    }
}
