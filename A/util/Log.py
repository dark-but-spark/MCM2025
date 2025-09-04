from util.Time import Time
import os

# 终端颜色代码
class TerminalColors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

# 日志级别配置
LOG_LEVELS = {
    "DEBUG": {"color": TerminalColors.OKCYAN, "priority": 0},
    "INFO": {"color": TerminalColors.OKGREEN, "priority": 1},
    "WARNING": {"color": TerminalColors.WARNING, "priority": 2},
    "ERROR": {"color": TerminalColors.FAIL, "priority": 3},
    "CRITICAL": {"color": TerminalColors.FAIL + TerminalColors.BOLD, "priority": 4}
}

def ensure_log_directory():
    """确保日志目录存在"""
    if not os.path.exists("logs"):
        os.makedirs("logs")

def Message(msg: str, level: str = "INFO",client=True):
    """
    记录日志消息，带有终端颜色输出和文件记录。

    Args:
        msg (str): 要记录的消息。
        level (str): 日志级别（如 "DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"）。
    """
    # 确保日志目录存在
    ensure_log_directory()
    
    # 获取当前时间
    time = Time.now()
    timestamp = time.format("%Y-%m-%d %H:%M:%S.%f")
    day = time.format("%Y-%m-%d")
    
    # 获取日志级别配置
    level_config = LOG_LEVELS.get(level.upper(), LOG_LEVELS["INFO"])
    color = level_config["color"]

    log_msg = f"[{timestamp}] [{level}] {msg}"
    
    # 写入日志文件
    with open(f"logs/{day}.log", "a", encoding="utf-8") as f:
        f.write(log_msg + "\n")
    if(client):
        print(f"{color}{log_msg}{TerminalColors.ENDC}")

# 示例使用
if __name__ == "__main__":
    Message("这是一条调试信息", "DEBUG")
    Message("系统正常运行", "INFO")
    Message("磁盘空间不足", "WARNING")
    Message("文件读取失败", "ERROR")
    Message("系统崩溃，立即退出", "CRITICAL")