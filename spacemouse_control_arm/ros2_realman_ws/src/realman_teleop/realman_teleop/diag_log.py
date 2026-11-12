"""
诊断日志工具 — VR 遥操作系统。

每次启动创建一个带时间戳的会话目录：
    ros2_realman_ws/logs/2026-02-23_14-30-22/
        dataset_recorder.log
        realman_driver.log
        ...

同一进程内所有 logger 共享同一个会话目录。

用法:
    from .diag_log import create_diag_logger
    _diag_log = create_diag_logger("dataset_recorder")
    _diag_log("some message")
"""

import os
import time
import threading

# 日志基础目录：ros2_realman_ws/logs/
_LOGS_BASE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))),
    "logs",
)

# 会话目录（同一进程内共享）
_session_dir: str | None = None
_session_lock = threading.Lock()


def _get_session_dir() -> str:
    """获取或创建会话日志目录（每个进程只创建一次）。"""
    global _session_dir
    if _session_dir is not None:
        return _session_dir
    with _session_lock:
        if _session_dir is not None:
            return _session_dir
        ts = time.strftime("%Y-%m-%d_%H-%M-%S")
        _session_dir = os.path.join(_LOGS_BASE, ts)
        os.makedirs(_session_dir, exist_ok=True)
    return _session_dir


def get_session_dir() -> str:
    """公开接口：获取当前会话日志目录。"""
    return _get_session_dir()


def create_diag_logger(name: str, print_to_stdout: bool = False):
    """
    创建诊断日志记录器，写入 logs/<session>/<name>.log。

    Args:
        name: 日志文件名（不含 .log 后缀）
        print_to_stdout: 是否同时打印到 stdout（默认 False）

    Returns:
        可调用的 _diag_log(msg: str) 函数。
    """
    _file_handle = None

    def _diag_log(msg: str):
        nonlocal _file_handle
        if _file_handle is None:
            session_dir = _get_session_dir()
            log_path = os.path.join(session_dir, f"{name}.log")
            _file_handle = open(log_path, "a")
            _file_handle.write(f"{'='*60}\n")
            _file_handle.write(f"[{name}] Session started: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            _file_handle.write(f"{'='*60}\n")
        line = f"[{time.strftime('%H:%M:%S')}.{int(time.monotonic()*1000)%1000:03d}] {msg}\n"
        _file_handle.write(line)
        _file_handle.flush()
        if print_to_stdout:
            print(msg)

    return _diag_log
