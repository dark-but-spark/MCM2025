// Rust version of Fly.py
pub struct Fly {
    pub vx: f64,
    pub vy: f64,
    pub vz: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}
const FY: [[f64; 3]; 6] = [
    [0.0, 0.0, 0.0],
    [17800.0, 0.0, 1800.0],
    [12000.0, 1400.0, 1400.0],
    [6000.0, -3000.0, 700.0],
    [11000.0, 2000.0, 1800.0],
    [13000.0, -2000.0, 1300.0],
];

impl Fly {
    pub fn new(direction: f64, v: f64, x: f64, y: f64, z: f64) -> Self {
        Fly { vx: v * direction.cos(), vy: v * direction.sin(), vz: 0.0, x, y, z }
    }
    pub fn new_id(id:usize,direction: f64, v: f64) -> Self {

        let idx = if id < FY.len() { id } else { 0 };
        let pos = FY[idx];
        Fly {
            vx: v * direction.cos(),
            vy: v * direction.sin(),
            vz: 0.0,
            x: pos[0],
            y: pos[1],
            z: pos[2],
        }
    }


    pub fn fly_tick(&mut self, t: f64) {
        self.x += self.vx * t;
        self.y += self.vy * t;
    }
    pub fn fly(&mut self,t:f64)->Fly {
        Fly {
            vx: self.vx,
            vy: self.vy,
            vz: self.vz,
            x: self.x + self.vx * t,
            y: self.y + self.vy * t,
            z: self.z,
        }
    }
    pub fn drop_tick(&mut self, t: f64) {
        self.x += self.vx * t;
        self.y += self.vy * t;
        self.z = self.z + self.vz * t - 0.5 * 9.81 * t * t;
        self.vz -= 9.81 * t;
    }
    pub fn drop(&mut self,t:f64)->Fly {
        let new_z = self.z + self.vz * t - 0.5 * 9.81 * t * t;
        let new_vz = self.vz - 9.81 * t;
        Fly {
            vx: self.vx,
            vy: self.vy,
            vz: new_vz,
            x: self.x + self.vx * t,
            y: self.y + self.vy * t,
            z: new_z,
        }
    }

}
