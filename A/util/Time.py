# Time.py

import datetime
from typing import Union, Optional

class Time:
    """
    一个处理时间戳并支持时间运算的类。
    可以自动将时间戳（秒或毫秒）转换为可读格式，并支持加减和比较运算。
    """

    def __init__(self, timestamp: Optional[Union[int, float]] = None, *, milliseconds: bool = False):
        """
        初始化 Time 对象。

        Args:
            timestamp (int, float, optional): 时间戳。如果为 None，则使用当前时间。
                默认假设为秒（如果小数部分小于1秒）或毫秒（如果整数部分很大）。可以通过 `milliseconds` 参数指定。
            milliseconds (bool): 如果为 True，则 `timestamp` 被解释为毫秒；否则解释为秒。默认为 False。

        Raises:
            ValueError: 如果时间戳无效。
        """
        if timestamp is None:
            # 使用当前时间
            self._timestamp_seconds = datetime.datetime.now().timestamp()
        else:
            # 判断时间戳单位并转换为秒
            if milliseconds:
                self._timestamp_seconds = timestamp / 1000.0
            else:
                # 尝试智能判断：如果数值很大（比如大于 10^10），可能是毫秒
                # 这个判断基于 Unix 时间戳在 2038 年才会达到 2^31 (约 21 亿)
                if isinstance(timestamp, (int, float)) and timestamp > 10**10:
                    print(f"Warning: Large timestamp {timestamp} detected, assuming milliseconds.")
                    self._timestamp_seconds = timestamp / 1000.0
                else:
                    self._timestamp_seconds = float(timestamp)

        # 验证时间戳的有效性 (防止太早或太晚)
        try:
            datetime.datetime.fromtimestamp(self._timestamp_seconds)
        except (OSError, ValueError) as e:
            raise ValueError(f"Invalid timestamp: {self._timestamp_seconds}") from e

    @classmethod
    def from_seconds(cls, seconds: float) -> 'Time':
        """
        从秒为单位的时间戳创建 Time 对象。

        Args:
            seconds (float): 以秒为单位的时间戳。

        Returns:
            Time: 新的 Time 实例。
        """
        return cls(seconds, milliseconds=False)

    @classmethod
    def from_milliseconds(cls, milliseconds: int) -> 'Time':
        """
        从毫秒为单位的时间戳创建 Time 对象。

        Args:
            milliseconds (int): 以毫秒为单位的时间戳。

        Returns:
            Time: 新的 Time 实例。
        """
        return cls(milliseconds, milliseconds=True)

    @classmethod
    def now(cls) -> 'Time':
        """
        创建一个表示当前时间的 Time 对象。

        Returns:
            Time: 当前时间的 Time 实例。
        """
        return cls()

    def to_seconds(self) -> float:
        """
        获取以秒为单位的时间戳。

        Returns:
            float: 秒级时间戳。
        """
        return self._timestamp_seconds

    def to_milliseconds(self) -> int:
        """
        获取以毫秒为单位的时间戳。

        Returns:
            int: 毫秒级时间戳。
        """
        return int(self._timestamp_seconds * 1000)

    def to_datetime(self) -> datetime.datetime:
        """
        转换为 datetime.datetime 对象。

        Returns:
            datetime.datetime: 对应的 datetime 对象。
        """
        return datetime.datetime.fromtimestamp(self._timestamp_seconds)

    def format(self, fmt: str = "%Y-%m-%d %H:%M:%S.%f") -> str:
        """
        格式化时间输出。

        Args:
            fmt (str): 格式字符串。默认包含毫秒 (%f)。

        Returns:
            str: 格式化后的时间字符串。
        """
        # 处理 %f (微秒) 到毫秒的转换
        dt = self.to_datetime()
        if "%f" in fmt:
            # 将微秒转换为毫秒并格式化
            milliseconds = dt.microsecond // 1000
            formatted = dt.strftime(fmt)
            return formatted.replace(f"{dt.microsecond:06d}", f"{milliseconds:03d}")
        else:
            return dt.strftime(fmt)

    def __str__(self) -> str:
        """
        返回可读的时间字符串表示（包含毫秒）。

        Returns:
            str: 格式化的字符串。
        """
        return self.format("%Y-%m-%d %H:%M:%S.%f")[:-3]  # 去掉最后3个0（微秒->毫秒）

    def __repr__(self) -> str:
        """
        返回对象的字符串表示。

        Returns:
            str: 包含类名和时间戳的字符串。
        """
        return f"Time({self._timestamp_seconds:.6f})"

    # 加法运算
    def __add__(self, other: Union['Time', float, int]) -> 'Time':
        """
        支持 Time + Time 或 Time + seconds。

        Args:
            other (Time, float, int): 要加上的 Time 对象或秒数。

        Returns:
            Time: 新的 Time 对象，表示相加后的时间。
        """
        if isinstance(other, Time):
            # 将两个时间戳相加没有实际意义，通常我们想加的是时间间隔（秒）
            # 这里按加秒数处理（即 other 的总秒数）
            new_timestamp = self._timestamp_seconds + other._timestamp_seconds
        elif isinstance(other, (int, float)):
            new_timestamp = self._timestamp_seconds + float(other)
        else:
            return NotImplemented
        return Time(new_timestamp)

    def __radd__(self, other: Union['Time', float, int]) -> 'Time':
        """
        支持 seconds + Time (反射加法)。

        Args:
            other (Time, float, int): 要加上的 Time 对象或秒数。

        Returns:
            Time: 新的 Time 对象，表示相加后的时间。
        """
        return self.__add__(other)

    # 减法运算
    def __sub__(self, other: Union['Time', float, int]) -> Union['Time', float]:
        """
        支持 Time - Time (返回秒数差) 或 Time - seconds (返回新的 Time)。

        Args:
            other (Time, float, int): 要减去的 Time 对象或秒数。

        Returns:
            Time or float: 如果减去 Time，返回秒数差 (float)；
                          如果减去秒数，返回新的 Time 对象。
        """
        if isinstance(other, Time):
            # Time - Time 返回时间差（秒）
            return self._timestamp_seconds - other._timestamp_seconds
        
        
if __name__ == "__main__":
    # 测试 Time 类
    t1 = Time.now()
    print("当前时间:", t1)
    
    t2 = Time.from_seconds(1633072800)  # 2021-10-01 00:00:00 UTC
    print("指定时间:", t2)
    
    t3 = t1 + 3600  # 加1小时
    print("加1小时后的时间:", t3)
    
    t4 = t2 - 3600  # 减1小时
    print("减1小时后的时间:", t4)
    
    print("t1 和 t2 的秒差:", t1 - t2)  # 秒差