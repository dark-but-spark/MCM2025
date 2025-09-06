use chrono::Local;
use std::fs::{create_dir_all, OpenOptions};
use std::io::Write;
use std::path::Path;

pub enum LogLevel {
    Debug,
    Info,
    Warning,
    Error,
    Critical,
}
impl LogLevel {
    fn as_str(&self) -> &'static str {
        match self {
            LogLevel::Debug => "DEBUG",
            LogLevel::Info => "INFO",
            LogLevel::Warning => "WARNING",
            LogLevel::Error => "ERROR",
            LogLevel::Critical => "CRITICAL",
        }
    }
    fn color(&self) -> &'static str {
        match self {
            LogLevel::Debug => "\x1b[96m",
            LogLevel::Info => "\x1b[92m",
            LogLevel::Warning => "\x1b[93m",
            LogLevel::Error => "\x1b[91m",
            LogLevel::Critical => "\x1b[1;91m",
        }
    }
}

fn ensure_log_directory() {
    let path = Path::new("logs");
    if !path.exists() {
        let _ = create_dir_all(path);
    }
}

pub fn message(msg: &str, level: LogLevel, client: bool) {
    ensure_log_directory();
    let now = Local::now();
    let timestamp = now.format("%Y-%m-%d %H:%M:%S%.3f").to_string();
    let day = now.format("%Y-%m-%d").to_string();
    let log_msg = format!("[{}] [{}] {}", timestamp, level.as_str(), msg);

    let log_path = format!("logs/{}.log", day);
    let mut file = OpenOptions::new()
        .create(true)
        .append(true)
        .open(&log_path)
        .unwrap();
    writeln!(file, "{}", log_msg).unwrap();

    if client {
        println!("{}{}{}", level.color(), log_msg, "\x1b[0m");
    }
}

pub fn main() {
    message("这是一条调试信息", LogLevel::Debug, true);
    message("系统正常运行", LogLevel::Info, true);
    message("磁盘空间不足", LogLevel::Warning, true);
    message("文件读取失败", LogLevel::Error, true);
    message("系统崩溃，立即退出", LogLevel::Critical, true);
}