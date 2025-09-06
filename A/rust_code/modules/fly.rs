// Rust version of Fly.py
#[derive(Debug, Clone)]
pub struct Fly {
    pub id: usize,
    pub direction: f64,
    pub v: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Fly {
    pub fn new(id: usize, direction: f64, v: f64) -> Self {
        // 初始化位置为0
        Fly { id, direction, v, x: 0.0, y: 0.0, z: 0.0 }
    }
    // 示例方法：飞行一段时间
    pub fn fly(&mut self, t: f64) {
        self.x += self.v * t * self.direction.cos();
        self.y += self.v * t * self.direction.sin();
        // z轴可根据实际需求调整
    }
    // 示例方法：投放
    pub fn drop(&mut self, t: f64) {
        self.z += t;
    }
}
