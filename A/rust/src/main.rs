use crate::modules::cylinder::Cylinder;
use crate::modules::missile::Missile;
use crate::modules::smokeball::SmokeBall;
use crate::modules::fly::Fly;
use Log::LogLevel;
use Log::message;
mod Log;
mod modules;
mod checker;
use once_cell::sync::Lazy;

static real: Lazy<Cylinder> = Lazy::new(|| Cylinder::new(7.0, 10.0, 0.0, 200.0, 0.0));

fn work(direction: f64,speed: f64,t1:f64,t2:f64,fyid: usize,mid: usize) -> (f64,f64,f64){
    let mut FY=Fly::new_id(fyid,direction,speed).fly(t1).drop(t2);
    let mut M=Missile::new_id(mid,300.0).time(t1+t2);
    let mut smoke_ball=SmokeBall::new(FY.x,FY.y,FY.z,10.0,-3.0);
    let mut t=t1+t2;
    let mut LL=t1+t2;let mut LR=t1+t2;
    let mut RL=t1+t2;let mut RR=t1+t2+20.0;
    loop{
        t+=0.1;
        M.time_tick(0.1);
        smoke_ball.time_tick(0.1);
        if (!checker::visible(&real,&smoke_ball,&M)){
            LL=t-0.1;LR=t;
            RL=t;
            break;
        }
        if t>t1+t2+20.0{
            return (-1.0,-1.0,-1.0);
        }
    }
    let mut L=LR;
    while LR-LL>1e-5 
    {
        let t=(LL+LR)/2.0;
        M=Missile::new_id(mid,300.0).time(t);
        smoke_ball=SmokeBall::new(FY.x,FY.y,FY.z,10.0,-3.0).time(t-t1-t2);
        if (!checker::visible(&real,&smoke_ball,&M)){
            L=t;
            LR=t;
        }else{
            LL=t;
        }
    }
    let mut R=RL;
    while RR-RL>1e-5
    {
        let t=(RR+RL)/2.0;
        M=Missile::new_id(mid,300.0).time(t);
        smoke_ball=SmokeBall::new(FY.x,FY.y,FY.z,10.0,-3.0).time(t-t1-t2);
        if (!checker::visible(&real,&smoke_ball,&M)){
            R=t;
            RL=t;
        }else{
            RR=t;
        }
    }
    message(&format!("fyid={} mid={} direction={} speed={} t1={} t2={} L={} R={}",fyid,mid,direction,speed,t1,t2,L,R),LogLevel::Info,true);

    (L,R,R-L)
}

fn main(){
    message("Problem1 start", LogLevel::Info, true);
    let (l,r,time)=work(-std::f64::consts::PI,120.0,1.5,3.6,1,1);
    message(&format!("L={}, R={}, R-L={}", l, r, time), LogLevel::Info, true);
    message("Problem1 end", LogLevel::Info, true);
}

fn test2(){
    let mut FY1=Fly::new(0.0,1.0,0.0,0.0,0.0);
    let mut FY2=Fly::new(0.0,1.0,0.0,0.0,0.0).fly(1.0);
    let mut FY3=Fly::new(0.0,1.0,0.0,0.0,0.0).fly(1.0).drop(1.0);
    message(&format!("FY1=({:.1}, {:.1}, {:.1})",FY1.x,FY1.y,FY1.z),LogLevel::Info,true);
    message(&format!("FY2=({:.1}, {:.1}, {:.1})",FY2.x,FY2.y,FY2.z),LogLevel::Info,true);
    message(&format!("FY3=({:.1}, {:.1}, {:.1})",FY3.x,FY3.y,FY3.z),LogLevel::Info,true);

}
fn test1() {
    let real2 = Cylinder::new(7.0, 10.0, 0.0, 50.0, 0.0);
    let mut M1 = Missile::new(100.0, -100.0, 0.0, 1.0);
    let smoke_ball = SmokeBall::new(50.0, -50.0, 0.0, 10.0, 0.0);
    let mut t = 0.0;
    let dt = 0.1;

    Log::message("startTest", LogLevel::Info, true);

    loop {
        M1.time_tick(dt);
        t += dt;
        Log::message(&format!("t={:.1} M1=({:.1}, {:.1}, {:.1})", t, M1.x, M1.y, M1.z), LogLevel::Debug, false);
        if checker::visible(&real2, &smoke_ball, &M1) {
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