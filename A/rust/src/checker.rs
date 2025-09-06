// Rust version of Checker.py
use crate::modules::{cylinder::Cylinder, smokeball::SmokeBall, missile::Missile};
use nalgebra::Vector3;

pub fn cross(A:(f64,f64,f64),O: &SmokeBall,M: &Missile)->bool{
    let AO=Vector3::new(O.x-A.0,O.y-A.1,O.z-A.2);
    let AM=Vector3::new(M.x-A.0,M.y-A.1,M.z-A.2);
    if AO.norm()<=O.r{
        return true;
    }
    let cosOAM=AO.dot(&AM)/(AO.norm()*AM.norm());
    let sinOAM=(1.0-cosOAM*cosOAM).sqrt();
    let d=AO.norm()*sinOAM;
    let l=AO.norm()*cosOAM;
    d<=O.r && AM.norm()>=l-(O.r*O.r-d*d).sqrt()
}

pub fn visible(check_cylinder: &Cylinder, smoke_ball: &SmokeBall, missile: &Missile) -> bool {
    let OM=Vector3::new(missile.x-smoke_ball.x,missile.y-smoke_ball.y,missile.z-smoke_ball.z);
    if OM.norm()<=smoke_ball.r{
        return false;
    }
    if missile.x<0.0{
        return true;
    }
    let checkN=500;
    for _i in 0..checkN{
        let point=check_cylinder.random_point_on_surface();
        if !cross(point,smoke_ball,missile){
            return true;
        }
    }
    false
}
