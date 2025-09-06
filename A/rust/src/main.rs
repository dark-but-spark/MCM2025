use crate::modules::cylinder::Cylinder;
use crate::modules::missile::Missile;
use crate::modules::smokeball::SmokeBall;
use Log::LogLevel;
mod Log;
mod modules;
mod checker;


fn main(){

    
}

fn test() {
    let real = Cylinder::new(7.0, 10.0, 0.0, 50.0, 0.0);
    let mut M1 = Missile::new(100.0, -100.0, 0.0, 1.0);
    let smoke_ball = SmokeBall::new(50.0, -50.0, 0.0, 10.0, 0.0);
    let mut t = 0.0;
    let dt = 0.1;

    Log::message("startTest", LogLevel::Info, true);

    loop {
        M1.time_tick(dt);
        t += dt;
        Log::message(&format!("t={:.1} M1=({:.1}, {:.1}, {:.1})", t, M1.x, M1.y, M1.z), LogLevel::Debug, false);
        if checker::visible(&real, &smoke_ball, &M1) {
            Log::message(&format!("t={:.1} M1=({:.1}, {:.1}, {:.1}) visible", t, M1.x, M1.y, M1.z), LogLevel::Warning, true);
        } else {
            Log::message(&format!("t={:.1} M1=({:.1}, {:.1}, {:.1}) hidden", t, M1.x, M1.y, M1.z), LogLevel::Info, true);
        }
        if M1.x < -5.0 {
            break;
        }
    }

    Log::message("endTest", LogLevel::Info, true);
}