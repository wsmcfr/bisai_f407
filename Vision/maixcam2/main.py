"""MaixCAM2 颜色二值化调参与追踪抓取基础程序。

运行方式：
1. 把 `main.py` 和 `vision_core.py` 放到 MaixCAM2 同一目录；
2. 在 MaixPy v4 环境运行本文件；
3. 先在 TUNE 页面调 LAB 阈值，二值化画面稳定后切到 TRACK 页面。
"""

from dataclasses import dataclass
import json

try:
    from .vision_core import (
        BlobCandidate,
        I2C_SLAVE_ADDR,
        LongPressAdjustState,
        THRESHOLD_FIELDS,
        ThresholdConfig,
        VisionTarget,
        consume_long_press_adjust_step,
        map_display_point_to_image,
        pack_i2c_result_frame,
        pick_best_blob_with_continuity,
        pick_largest_valid_blob,
    )
except ImportError:
    from vision_core import (
        BlobCandidate,
        I2C_SLAVE_ADDR,
        LongPressAdjustState,
        THRESHOLD_FIELDS,
        ThresholdConfig,
        VisionTarget,
        consume_long_press_adjust_step,
        map_display_point_to_image,
        pack_i2c_result_frame,
        pick_best_blob_with_continuity,
        pick_largest_valid_blob,
    )


# MaixPy 相关模块只在真机运行时导入，便于普通 Python 环境做语法检查。
from maix import app, camera, display, image, time, touchscreen


CONFIG_PATH = "vision_settings.json"
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
# 最小有效色块面积。原值 120 在 320x240 下相当于 0.16% 面积，背景同色碎片很容易超过；
# 提到 220 后基本能滤掉地面/桌面/墙面级别的低饱和干扰。
MIN_PIXELS = 220
# 允许上报的最大偏差。半屏宽 160px，140px 留约 20px 安全裕量；
# 原值 220 大于半屏宽，导致这条限制对 320x240 图像几乎无效。
MAX_OFFSET_PX = 140

# 平滑 alpha 三段自适应：静止时极慢平滑用来压制 ±2 像素级别的底层噪声；
# 慢动用常规平滑保证目标跟随；急动用高响应平滑保证大幅运动不落后。
TARGET_SMOOTH_ALPHA_DEN = 100
TARGET_SMOOTH_ALPHA_STILL_NUM = 20
TARGET_SMOOTH_ALPHA_SLOW_NUM = 55
TARGET_SMOOTH_ALPHA_FAST_NUM = 85
TARGET_MOTION_STILL_PX = 8
TARGET_MOTION_SLOW_PX = 35

# 跨帧锁定阈值。连续看到 N 帧才视为锁定并上报 found=True，避免单帧误识别让机械臂动作；
# 丢失 M 帧之内仍按缓存输出虚拟目标，避免一帧丢失就跳一次。
TARGET_LOCK_FRAMES = 2
TARGET_LOST_HOLD_FRAMES = 3

# 锁定后下一帧 ROI 半宽/半高，单位像素。覆盖目标本身 + 60% 余量，
# 既能减少全图扫描带来的背景干扰，也能容忍目标快速横向移动。
TARGET_ROI_HALF_PX = 60

THRESHOLD_STEP = 1
LONG_PRESS_REPEAT_MS = 100
I2C_FREQ_HZ = 50000

# 当前先切到 MaixCAM2 官方示例常用的 I2C6 引脚组合做交叉验证。
# 官方示例方向为：`A1 -> I2C6_SCL`，`A0 -> I2C6_SDA`。
I2C_BUS_ID = 6
I2C_SCL_PIN_NAME = "A1"
I2C_SDA_PIN_NAME = "A0"
I2C_SCL_FUNC_NAME = "I2C6_SCL"
I2C_SDA_FUNC_NAME = "I2C6_SDA"
I2C_STATUS_STALE_MS = 300
I2C_SEND_INTERVAL_MS = 25
I2C_ERROR_RETRY_MS = 200
I2C_SCAN_INTERVAL_MS = 800
I2C_SCAN_ERROR_RETRY_MS = 3000
I2C_SCAN_MISSING_ADDR_BACKOFF_MS = 5000
I2C_TRACK_ENTER_DIAG_DELAY_MS = 150
I2C_WORKER_LOOP_SLEEP_MS = 20
I2C_PINMAP_LOGGED = False

MODE_TUNE = "tune"
MODE_TRACK = "track"

COLOR_BLACK = image.COLOR_BLACK
COLOR_WHITE = image.COLOR_WHITE
COLOR_RED = image.COLOR_RED
COLOR_GREEN = image.COLOR_GREEN
COLOR_BLUE = image.COLOR_BLUE
COLOR_YELLOW = image.COLOR_YELLOW

TRACK_BUTTON = (250, 4, 66, 30)
TUNE_BUTTON = (4, 4, 58, 30)
LINK_BUTTON = (4, 40, 58, 30)
PLUS_BUTTON = (268, 54, 48, 48)
MINUS_BUTTON = (268, 112, 48, 48)
FIELD_BUTTON_Y = 198
FIELD_BUTTON_H = 38
FIELD_BUTTON_W = 53


@dataclass
class I2CCommState:
    """记录 MaixCAM2 本机侧 I2C 写入状态。

    说明：
    - 这里的“通讯成功”定义为“MaixCAM2 本帧对 I2C 从机写入成功”；
    - 当前协议是单向发给 ESP32，没有应用层回包，所以不能仅靠这一侧判断
      ESP32 业务逻辑是否已经消费完该帧。
    """

    bus_available: bool = False       # I2C 主机对象是否初始化成功；False 时界面显示 `I2C OFF`。
    last_write_ok: bool | None = None # 最近一次写入是否成功；None 表示还没有真正发送过。
    last_ok_ms: int | None = None     # 最近一次写成功的时间戳，单位毫秒，用于区分 `OK` 和 `WAIT`。
    last_attempt_ms: int | None = None # 最近一次尝试发送的时间戳，单位毫秒，便于后续扩展调试信息。
    last_write_length: int | None = None # 最近一次 `writeto()` 的返回值；负数通常表示底层 I2C 错误码。
    last_error_text: str = ""         # 最近一次写失败的异常文本，仅保留短日志，不直接显示到屏幕。
    last_scan_ok: bool | None = None  # 最近一次 `scan()` 是否成功；None 表示还没扫过总线。
    last_scan_ms: int | None = None   # 最近一次总线扫描时间戳，单位毫秒。
    last_scan_addresses: tuple[int, ...] = () # 最近一次扫描到的 7 位地址列表，便于界面显示和排障。
    target_addr_visible: bool | None = None # 最近一次扫描时是否看到了目标地址 `0x42`。
    last_scan_error_text: str = ""    # 最近一次扫描失败的异常文本，仅打印日志，不直接铺满屏幕。
    link_enabled: bool = False        # 当前是否允许后台线程尝试 I2C；False 时界面显示 `I2C OFF`。


@dataclass
class I2CWorkerState:
    """记录后台 I2C 线程与主显示线程之间的共享状态。

    说明：
    - 主线程只负责更新最新视觉结果帧和模式开关；
    - 后台线程独占 I2C 总线对象，避免阻塞式 `scan()/writeto()` 卡住显示；
    - 这里只共享简单字段，尽量降低线程间同步复杂度。
    """

    thread_available: bool = False    # MaixPy `maix.thread` 是否可用；不可用时退回到“只显示不通讯”模式。
    running: bool = False             # 后台线程运行标志；主线程退出前会置 False 请求收尾。
    track_mode_active: bool = False   # True 表示当前处于 TRACK 模式，后台线程才会真的访问 I2C。
    link_enabled: bool = False        # 用户是否主动打开 I2C 链路；False 时后台线程完全不碰 I2C。
    force_reprobe: bool = False       # 主线程切到 TRACK 后置 True，要求后台线程重建并重新探测总线。
    pending_frame: bytes = b""        # 主线程最近一次生成的视觉结果帧；后台线程始终只发送最新一帧。
    thread_obj: object | None = None  # MaixPy 线程对象引用，避免被解释器提前回收。


@dataclass
class TargetSmoothState:
    """记录 TRACK 页面目标平滑状态。

    说明：
    - MaixCAM2 的色块检测会受光照、反光和二值化边缘影响，同一个物体的中心点会轻微跳动；
    - 这里对 `cx/cy/area` 做三段自适应指数平滑，既保留目标从左到右移动时的大幅偏差，
      又避免单帧噪声让 ESP32 机械臂控制来回抖；
    - 该状态只在主循环中读写，不跨线程共享，I2C 后台线程只拿最终打包后的二进制帧；
    - `locked_streak` 用来要求"连续看到 N 帧才上报 found=True"，避免单帧误识别让机械臂动作；
    - `lost_streak` 用来支持"丢失 M 帧内仍按上次平滑结果输出"，避免一帧丢失就跳一次。
    """

    has_target: bool = False      # 是否已有可复用的上一帧平滑目标；目标丢失超过保持窗口时置 False。
    cx: int = 0                   # 平滑后的目标中心 X 坐标，单位像素。
    cy: int = 0                   # 平滑后的目标中心 Y 坐标，单位像素。
    area: int = 0                 # 平滑后的目标面积，单位像素。
    dx_px: int = 0                # 平滑后的 X 偏差，右正左负，单位像素。
    dy_px: int = 0                # 平滑后的 Y 偏差，下正上负，单位像素。
    last_w: int = 20              # 最近一次实际看到目标时的外接矩形宽度，仅用于丢失保持时还原 VisionTarget。
    last_h: int = 20              # 最近一次实际看到目标时的外接矩形高度。
    locked_streak: int = 0        # 当前已连续看到目标的帧数；用于满足 TARGET_LOCK_FRAMES 才视为锁定。
    lost_streak: int = 0          # 当前已连续丢失目标的帧数；用于决定是否仍在丢失保持窗口内。


def point_in_rect(x_pos, y_pos, rect):
    """判断触摸点是否落在矩形控件内。

    参数：
    - x_pos/y_pos：触摸点坐标，单位像素；
    - rect：`(x, y, w, h)` 控件矩形。

    返回值：
    - 命中控件返回 True，否则返回 False。

    副作用：无。该函数只服务 UI 命中测试。
    """
    rect_x, rect_y, rect_w, rect_h = rect
    return (rect_x <= x_pos < rect_x + rect_w) and (rect_y <= y_pos < rect_y + rect_h)


def get_field_button_rect(field_index):
    """计算底部阈值字段按钮矩形。

    参数：
    - field_index：`THRESHOLD_FIELDS` 中的字段序号。

    返回值：
    - 返回 `(x, y, w, h)`，用于触摸命中和绘制按钮。

    副作用：无。最后一个按钮会吃掉除法余量，避免右侧出现不可点击窄缝。
    """
    button_x = field_index * FIELD_BUTTON_W
    button_w = CAMERA_WIDTH - button_x if field_index == len(THRESHOLD_FIELDS) - 1 else FIELD_BUTTON_W
    return (button_x, FIELD_BUTTON_Y, button_w, FIELD_BUTTON_H)


def draw_text(img, x_pos, y_pos, text, color=COLOR_WHITE, scale=1.0):
    """在图像上绘制短文本。

    参数：
    - img：MaixPy 图像对象；
    - x_pos/y_pos：文本左上角坐标；
    - text：需要显示的短文本；
    - color：文本颜色；
    - scale：字体缩放倍率。

    返回值：无。
    副作用：会修改传入图像的显示内容。
    """
    img.draw_string(x_pos, y_pos, str(text), color=color, scale=scale)


def draw_button(img, rect, label, selected=False):
    """绘制一个稳定尺寸的触摸按钮。

    参数：
    - img：MaixPy 图像对象；
    - rect：`(x, y, w, h)` 按钮区域；
    - label：按钮显示文本；
    - selected：True 时用黄色边框突出当前选中项。

    返回值：无。
    副作用：会在画面上覆盖按钮背景和文字。
    """
    rect_x, rect_y, rect_w, rect_h = rect
    border_color = COLOR_YELLOW if selected else COLOR_WHITE
    img.draw_rect(rect_x, rect_y, rect_w, rect_h, color=border_color, thickness=2)
    label_lines = str(label).split("\n")
    for index, line_text in enumerate(label_lines):
        draw_text(img, rect_x + 4, rect_y + 6 + index * 14, line_text, color=border_color, scale=0.8)


def load_threshold_config():
    """从本地 JSON 文件加载 LAB 阈值。

    参数：无。
    返回值：`ThresholdConfig` 对象；文件不存在或内容非法时返回默认黄色阈值。
    副作用：会读取 `CONFIG_PATH`，不会在失败时抛出异常，避免 MaixCAM2 上电无法启动。
    """
    try:
        with open(CONFIG_PATH, "r") as config_file:
            raw_config = json.load(config_file)
        return ThresholdConfig.from_dict(raw_config)
    except Exception as exc:
        print("[VISION] use default threshold:", exc)
        return ThresholdConfig()


def save_threshold_config(config):
    """把当前 LAB 阈值保存到本地 JSON 文件。

    参数：
    - config：当前正在使用的阈值对象。

    返回值：无。
    副作用：会写入 `CONFIG_PATH`；写入失败只打印日志，不影响继续追踪。
    """
    try:
        with open(CONFIG_PATH, "w") as config_file:
            json.dump(config.to_dict(), config_file)
    except Exception as exc:
        print("[VISION] save threshold failed:", exc)


def create_i2c_or_none():
    """尝试创建 MaixCAM2 到 ESP32 的 I2C 主机链路。

    参数：无。
    返回值：
    - 成功时返回 MaixPy I2C 主机对象；
    - 引脚映射或外设初始化失败时返回 None。

    副作用：
    - 会把 MaixCAM2 当前选定的引脚复用到 `I2C6_SCL/I2C6_SDA`；
    - 手持调参时可以不接线，失败不会影响画面功能，只是 TRACK 模式发不出结果。
    """
    global I2C_PINMAP_LOGGED

    try:
        from maix import err, i2c, pinmap

        # 当前版本先切到 MaixCAM2 官方示例常用的 I2C6：
        # A1 -> I2C6_SCL, A0 -> I2C6_SDA。
        err.check_raise(
            pinmap.set_pin_function(I2C_SCL_PIN_NAME, I2C_SCL_FUNC_NAME),
            "set {} failed".format(I2C_SCL_FUNC_NAME),
        )
        err.check_raise(
            pinmap.set_pin_function(I2C_SDA_PIN_NAME, I2C_SDA_FUNC_NAME),
            "set {} failed".format(I2C_SDA_FUNC_NAME),
        )
        # MaixCAM2 官方 pinmap 文档说明该板支持读取当前引脚功能。
        # 这里把实际映射结果打印出来，便于串口日志确认 A0/A1 是否真的切到了 I2C6。
        if not I2C_PINMAP_LOGGED:
            # 只在第一次建链时打印 pinmap，避免 LINK 重试时刷屏。
            # 如果这里能打印出 `I2C6_SCL/I2C6_SDA`，说明 MaixCAM2 侧引脚复用已经成功。
            try:
                print("[VISION] I2C init bus={} freq={}Hz".format(I2C_BUS_ID, I2C_FREQ_HZ))
                print("[VISION] {} => {}".format(I2C_SCL_PIN_NAME, pinmap.get_pin_function(I2C_SCL_PIN_NAME)))
                print("[VISION] {} => {}".format(I2C_SDA_PIN_NAME, pinmap.get_pin_function(I2C_SDA_PIN_NAME)))
            except Exception:
                pass
            I2C_PINMAP_LOGGED = True
        return i2c.I2C(I2C_BUS_ID, i2c.Mode.MASTER, freq=I2C_FREQ_HZ)
    except Exception as exc:
        print("[VISION] I2C disabled:", exc)
        return None


def prepare_track_i2c_state(comm_state, now_ms):
    """在切入 TRACK 模式时重置 I2C 诊断节奏。

    参数：
    - comm_state：I2C 状态缓存，会被原地清空诊断结果；
    - now_ms：当前毫秒时间戳。

    返回值：
    - 返回 `(next_scan_ms, next_write_ms)`，用于主循环安排首次探测和首次发送。

    设计原因：
    - 进入 TRACK 页的首帧优先保证画面先显示出来；
    - 把首次 I2C 诊断延后一个很短的时间窗口，避免用户点击切页时立刻撞上阻塞式总线探测。
    """
    delayed_ms = int(now_ms) + I2C_TRACK_ENTER_DIAG_DELAY_MS

    comm_state.last_write_ok = None
    comm_state.last_ok_ms = None
    comm_state.last_attempt_ms = None
    comm_state.last_write_length = None
    comm_state.last_error_text = ""
    comm_state.last_scan_ok = None
    comm_state.last_scan_ms = None
    comm_state.last_scan_addresses = ()
    comm_state.target_addr_visible = None
    comm_state.last_scan_error_text = ""
    return delayed_ms, delayed_ms


def set_i2c_worker_track_mode(worker_state, comm_state, now_ms, enabled):
    """通知后台 I2C 线程当前是否进入 TRACK 模式。

    参数：
    - worker_state：后台 I2C 线程共享状态；None 表示当前固件不支持线程方案。
    - comm_state：前台显示使用的 I2C 状态缓存。
    - now_ms：当前毫秒时间戳。
    - enabled：True 表示进入 TRACK，False 表示离开 TRACK。

    返回值：无。

    副作用：
    - 进入 TRACK 时默认只允许显示，不自动碰 I2C，避免硬件故障时一切页就卡；
    - 离开 TRACK 时会彻底关闭 I2C 链路，避免调参页仍在占用总线。
    """
    if worker_state is None:
        return

    worker_state.track_mode_active = bool(enabled)
    worker_state.link_enabled = False
    worker_state.force_reprobe = False
    worker_state.pending_frame = b""
    comm_state.link_enabled = False
    prepare_track_i2c_state(comm_state, now_ms)


def set_i2c_worker_link_enabled(worker_state, comm_state, now_ms, enabled):
    """手动打开或关闭 I2C 链路。

    参数：
    - worker_state：后台 I2C 线程共享状态；None 表示线程不可用。
    - comm_state：前台显示使用的 I2C 状态缓存。
    - now_ms：当前毫秒时间戳。
    - enabled：True 表示允许后台线程开始 I2C 探测；False 表示立即熔断停用。

    返回值：无。

    设计原因：
    - 只有用户主动点击 `LINK` 按钮时才尝试 I2C，避免接线错误时一进入 TRACK 就卡；
    - 一旦关闭链路，后台线程完全不再触碰 I2C，确保画面恢复流畅。
    """
    if worker_state is None:
        comm_state.link_enabled = False
        comm_state.bus_available = False
        comm_state.last_write_ok = False
        comm_state.last_write_length = None
        comm_state.last_error_text = "i2c worker unavailable"
        return

    worker_state.link_enabled = bool(enabled)
    worker_state.pending_frame = b""
    comm_state.link_enabled = bool(enabled)

    if enabled:
        worker_state.force_reprobe = True
        prepare_track_i2c_state(comm_state, now_ms)
        return

    worker_state.force_reprobe = False
    comm_state.bus_available = False
    comm_state.last_write_ok = None
    comm_state.last_write_length = None
    comm_state.last_error_text = ""
    comm_state.last_scan_ok = None
    comm_state.last_scan_ms = None
    comm_state.last_scan_addresses = ()
    comm_state.target_addr_visible = None
    comm_state.last_scan_error_text = ""


def submit_i2c_result_frame(worker_state, frame_bytes):
    """把最新视觉结果帧交给后台 I2C 线程异步发送。

    参数：
    - worker_state：后台 I2C 线程共享状态；None 表示线程不可用。
    - frame_bytes：已经打包好的固定长度视觉结果帧。

    返回值：无。

    设计原因：
    - 追踪抓取只关心“最新目标偏差”，不需要把每一帧都排队发完；
    - 直接覆盖 `pending_frame` 可以避免后台线程跟不上时堆积旧数据。
    """
    if (worker_state is None) or (not worker_state.track_mode_active):
        return

    worker_state.pending_frame = bytes(frame_bytes)


def scan_i2c_bus(i2c_bus, comm_state, now_ms):
    """扫描当前 I2C 总线并记录是否能看到目标地址。

    参数：
    - i2c_bus：`create_i2c_or_none()` 返回的 I2C 对象或 None；
    - comm_state：I2C 状态缓存，会原地写入扫描结果；
    - now_ms：当前毫秒时间戳。

    返回值：
    - 扫描成功返回 True；
    - 总线不可用或 `scan()` 抛异常返回 False。

    副作用：
    - 会更新 `last_scan_*` 字段；
    - 用于把“总线有没有真的看到 0x42”这件事直接暴露到 TRACK 页面。
    """
    comm_state.last_scan_ms = int(now_ms)
    comm_state.bus_available = i2c_bus is not None

    if i2c_bus is None:
        comm_state.last_scan_ok = False
        comm_state.last_scan_addresses = ()
        comm_state.target_addr_visible = False
        comm_state.last_scan_error_text = "i2c bus unavailable"
        return False

    try:
        # MaixPy 官方 I2C API 支持 `scan(addr)` 定向扫描单个 7 位地址。
        # 当前只关心 ESP32 从机 `0x42` 是否应答，优先定向扫描可以明显减少
        # 接线错误或从机掉线时全总线探测带来的阻塞时间。
        # 若用户设备固件较旧、不支持带参数 `scan(addr)`，下面会退回原来的全总线扫描。
        scan_mode = "target"
        try:
            raw_addresses = i2c_bus.scan(I2C_SLAVE_ADDR)
            # 如果定向扫描 0x42 没有应答，再做一次全总线扫描。
            # 这样能区分“ESP32 完全没有应答”和“设备存在但地址不是 0x42”。
            if not raw_addresses:
                scan_mode = "target+full"
                raw_addresses = i2c_bus.scan()
        except TypeError:
            scan_mode = "full"
            raw_addresses = i2c_bus.scan()
        normalized_addresses = []

        if raw_addresses is None:
            raw_addresses = []

        for raw_addr in raw_addresses:
            try:
                addr_value = int(raw_addr)
            except Exception:
                continue

            if (0 <= addr_value <= 0x7F) and (addr_value not in normalized_addresses):
                normalized_addresses.append(addr_value)

        normalized_addresses = tuple(sorted(normalized_addresses))
        comm_state.last_scan_ok = True
        comm_state.last_scan_addresses = normalized_addresses
        comm_state.target_addr_visible = I2C_SLAVE_ADDR in normalized_addresses
        comm_state.last_scan_error_text = ""
        # 这条日志直接回答“MaixCAM2 主机有没有在总线上看到 ESP32 地址 0x42”。
        # pinmap 成功但这里一直 False 时，问题通常在外部接线、ESP32 程序或从机地址。
        print(
            "[VISION] I2C scan bus={} mode={} addrs={} target=0x{:02X} found={}".format(
                I2C_BUS_ID,
                scan_mode,
                ["0x{:02X}".format(addr) for addr in normalized_addresses],
                I2C_SLAVE_ADDR,
                comm_state.target_addr_visible,
            )
        )
        return True
    except Exception as exc:
        comm_state.last_scan_ok = False
        comm_state.last_scan_addresses = ()
        comm_state.target_addr_visible = False
        comm_state.last_scan_error_text = str(exc)
        print("[VISION] I2C scan failed:", exc)
        return False


def can_write_i2c_result(comm_state):
    """判断当前是否允许向 ESP32 发送视觉帧。

    参数：
    - comm_state：I2C 状态缓存。

    返回值：
    - 只有最近一次 `scan()` 成功且确实看到了 `0x42` 时返回 True；
    - 其它情况一律返回 False。

    设计原因：
    - 在 `SCAN NONE` 或 `SCAN ERR` 场景继续 `writeto()`，只会让 MaixCAM2 卡顿更明显；
    - 先确保总线上真的可见目标地址，再进入持续发送阶段。
    """
    return (comm_state.last_scan_ok is True) and (comm_state.target_addr_visible is True)


def create_i2c_worker_or_none(comm_state):
    """创建专门负责 I2C 通讯的后台线程。

    参数：
    - comm_state：主线程用于显示状态的共享缓存。

    返回值：
    - 创建成功返回 `I2CWorkerState`；
    - 当前固件不支持线程，或线程创建失败时返回 None。

    设计原因：
    - MaixPy 官方提供 `maix.thread.Thread.detach()`，适合把阻塞式 I2C 从显示循环剥离；
    - 后台线程只碰 I2C，主线程只碰摄像头/显示/触摸，职责分离更稳。
    """
    try:
        from maix import thread as maix_thread
    except Exception as exc:
        print("[VISION] I2C worker disabled:", exc)
        return None

    worker_state = I2CWorkerState(thread_available=True, running=True)

    def worker_entry(_):
        """后台 I2C 线程主体。

        主要流程：
        1. 只在 TRACK 模式下工作，平时轻量休眠；
        2. 线程内部独占 I2C 总线对象，避免主线程直接进入阻塞式驱动调用；
        3. 先扫到 `0x42` 再发送结果帧，扫不到就长退避后重试；
        4. 始终只发送最新帧，确保机械臂拿到的是当前目标偏差。
        """
        i2c_bus = None
        next_i2c_scan_ms = 0
        next_i2c_write_ms = 0

        while worker_state.running:
            now_ms = time.ticks_ms()

            if worker_state.force_reprobe:
                next_i2c_scan_ms, next_i2c_write_ms = prepare_track_i2c_state(comm_state, now_ms)
                worker_state.force_reprobe = False
                i2c_bus = None

            if (not worker_state.track_mode_active) or (not worker_state.link_enabled):
                time.sleep_ms(I2C_WORKER_LOOP_SLEEP_MS)
                continue

            if (i2c_bus is None) and (int(now_ms) >= int(next_i2c_scan_ms)):
                i2c_bus = create_i2c_or_none()
                comm_state.bus_available = i2c_bus is not None

                if i2c_bus is None:
                    comm_state.last_scan_ok = False
                    comm_state.last_scan_addresses = ()
                    comm_state.target_addr_visible = False
                    comm_state.last_scan_error_text = "i2c bus unavailable"
                    comm_state.last_write_ok = False
                    comm_state.last_write_length = None
                    comm_state.last_error_text = "i2c bus unavailable"
                    next_i2c_scan_ms = int(now_ms) + I2C_SCAN_ERROR_RETRY_MS
                    next_i2c_write_ms = int(next_i2c_scan_ms)
                    time.sleep_ms(I2C_WORKER_LOOP_SLEEP_MS)
                    continue

            if (i2c_bus is not None) and (int(now_ms) >= int(next_i2c_scan_ms)):
                scan_ok = scan_i2c_bus(i2c_bus, comm_state, now_ms)
                if scan_ok:
                    if comm_state.target_addr_visible:
                        next_i2c_scan_ms = int(now_ms) + I2C_SCAN_INTERVAL_MS
                        if int(next_i2c_write_ms) < int(now_ms):
                            next_i2c_write_ms = int(now_ms)
                    else:
                        next_i2c_scan_ms = int(now_ms) + I2C_SCAN_MISSING_ADDR_BACKOFF_MS
                        next_i2c_write_ms = int(next_i2c_scan_ms)
                        # 一旦明确扫不到 `0x42`，立刻熔断停用本次链路尝试，
                        # 避免后台线程继续反复探测，把整机拖进持续卡顿。
                        worker_state.link_enabled = False
                        comm_state.link_enabled = False
                else:
                    i2c_bus = None
                    next_i2c_scan_ms = int(now_ms) + I2C_SCAN_ERROR_RETRY_MS
                    next_i2c_write_ms = int(next_i2c_scan_ms)
                    # 扫描异常也立即熔断；后续只有用户再次点击 `LINK` 才会重试。
                    worker_state.link_enabled = False
                    comm_state.link_enabled = False
                    time.sleep_ms(I2C_WORKER_LOOP_SLEEP_MS)
                    continue

            if (i2c_bus is not None) and can_write_i2c_result(comm_state) and (int(now_ms) >= int(next_i2c_write_ms)):
                frame_bytes = worker_state.pending_frame
                if frame_bytes:
                    write_ok = write_i2c_frame(i2c_bus, frame_bytes, comm_state, now_ms)
                    if write_ok:
                        next_i2c_write_ms = int(now_ms) + I2C_SEND_INTERVAL_MS
                    else:
                        next_i2c_write_ms = int(now_ms) + I2C_ERROR_RETRY_MS
                        # 写失败说明当前电气层或从机应答不稳定，本轮直接熔断停用，
                        # 避免后台线程继续持续写总线导致全局卡顿。
                        worker_state.link_enabled = False
                        comm_state.link_enabled = False
                else:
                    next_i2c_write_ms = int(now_ms) + I2C_SEND_INTERVAL_MS

            time.sleep_ms(I2C_WORKER_LOOP_SLEEP_MS)

    try:
        worker_state.thread_obj = maix_thread.Thread(worker_entry, None)
        worker_state.thread_obj.detach()
        return worker_state
    except Exception as exc:
        print("[VISION] I2C worker start failed:", exc)
        return None


def write_i2c_frame(i2c_bus, frame_bytes, comm_state, now_ms):
    """把视觉结果帧写到 ESP32 I2C 从机。

    参数：
    - i2c_bus：`create_i2c_or_none()` 返回的 I2C 对象或 None；
    - frame_bytes：已经打包好的固定长度视觉结果帧。
    - comm_state：I2C 通讯状态缓存，用于给追踪界面显示 `OK/ERR/OFF`；
    - now_ms：当前毫秒时间戳，建议直接传主循环里的 `time.ticks_ms()`。

    返回值：
    - 本次写入成功返回 True；
    - 写入失败或总线不可用返回 False。

    副作用：
    - TRACK 模式每帧调用一次；
    - 总线异常时只打印错误，不阻塞图像刷新。
    """
    comm_state.last_attempt_ms = int(now_ms)
    comm_state.bus_available = i2c_bus is not None

    if i2c_bus is None:
        comm_state.last_write_ok = False
        comm_state.last_write_length = None
        comm_state.last_error_text = "i2c bus unavailable"
        return False

    try:
        write_length = i2c_bus.writeto(I2C_SLAVE_ADDR, frame_bytes)
        comm_state.last_write_length = None if write_length is None else int(write_length)

        if (write_length is not None) and (int(write_length) != len(frame_bytes)):
            comm_state.last_write_ok = False
            if int(write_length) < 0:
                comm_state.last_error_text = "writeto returned {}".format(int(write_length))
                print("[VISION] I2C write returned error:", write_length)
            else:
                comm_state.last_error_text = "write length mismatch"
                print("[VISION] I2C write length mismatch:", write_length)
            return False

        comm_state.last_write_ok = True
        comm_state.last_ok_ms = int(now_ms)
        comm_state.last_error_text = ""
        return True
    except Exception as exc:
        comm_state.last_write_ok = False
        comm_state.last_write_length = None
        comm_state.last_error_text = str(exc)
        print("[VISION] I2C write failed:", exc)
        return False


def get_i2c_status_display(comm_state, now_ms, stale_ms=I2C_STATUS_STALE_MS):
    """根据当前 I2C 写入状态生成追踪页显示文本。

    参数：
    - comm_state：I2C 通讯状态缓存；
    - now_ms：当前毫秒时间戳；
    - stale_ms：最近一次成功写入在多少毫秒内可视为“仍然在线”。

    返回值：
    - 返回 `(status_text, status_color)` 二元组；
    - 文本只表示 MaixCAM2 这一侧的 I2C 写入是否成功，不代表 ESP32 业务层已完成处理。

    副作用：无。
    """
    if not comm_state.link_enabled:
        return "I2C OFF", COLOR_WHITE

    if not comm_state.bus_available:
        return "I2C OFF", COLOR_RED

    if comm_state.last_write_ok is None:
        return "I2C WAIT", COLOR_YELLOW

    if comm_state.last_write_ok:
        if (comm_state.last_ok_ms is not None) and ((int(now_ms) - comm_state.last_ok_ms) <= stale_ms):
            return "I2C OK", COLOR_GREEN
        return "I2C WAIT", COLOR_YELLOW

    return "I2C ERR", COLOR_RED


def get_i2c_scan_display(comm_state):
    """根据最近一次总线扫描结果生成 TRACK 页诊断文本。

    参数：
    - comm_state：I2C 状态缓存。

    返回值：
    - 返回 `(scan_text, scan_color)`；
    - 文本重点回答“总线上有没有看到 0x42”。

    副作用：无。
    """
    if not comm_state.link_enabled:
        return "SCAN OFF", COLOR_WHITE

    if not comm_state.bus_available:
        return "SCAN OFF", COLOR_RED

    if comm_state.last_scan_ok is None:
        return "SCAN WAIT", COLOR_YELLOW

    if not comm_state.last_scan_ok:
        return "SCAN ERR", COLOR_RED

    if comm_state.target_addr_visible:
        return "SCAN 42 OK", COLOR_GREEN

    if not comm_state.last_scan_addresses:
        return "SCAN NONE", COLOR_RED

    visible_text = ",".join("{:02X}".format(addr_value) for addr_value in comm_state.last_scan_addresses[:3])
    if len(comm_state.last_scan_addresses) > 3:
        visible_text += "+"
    return "SCAN {}".format(visible_text), COLOR_RED


def get_i2c_detail_display(comm_state):
    """生成 I2C 失败时的简短细节文本。

    参数：
    - comm_state：I2C 状态缓存。

    返回值：
    - 正常时返回 `("", COLOR_WHITE)`；
    - 失败时返回一行简短错误提示，例如 `RET -5` 或 `NO 42`。

    副作用：无。
    """
    if not comm_state.link_enabled:
        return "TAP LINK", COLOR_WHITE

    if comm_state.last_write_ok is not False:
        return "", COLOR_WHITE

    if comm_state.last_write_length is not None:
        return "RET {}".format(comm_state.last_write_length), COLOR_RED

    error_text = str(comm_state.last_error_text).upper()
    if ("0X42" in error_text) or ("NO 42" in error_text):
        return "NO 42", COLOR_RED
    if "NACK" in error_text:
        return "NACK", COLOR_RED
    if not error_text:
        return "WRITE ERR", COLOR_RED
    return error_text[:12], COLOR_RED


def blob_value(blob, method_name, index, default_value=0):
    """兼容不同 MaixPy blob 表示方式读取字段。

    参数：
    - blob：MaixPy `find_blobs` 返回的单个目标；
    - method_name：优先调用的方法名，如 `cx`；
    - index：方法不存在时尝试按序号读取；
    - default_value：两种读取方式都失败时使用的默认值。

    返回值：整数形式的字段值。
    副作用：无。该兼容层避免 MaixPy API 小版本差异导致主循环直接崩溃。
    """
    method = getattr(blob, method_name, None)
    if callable(method):
        try:
            return int(method())
        except Exception:
            pass

    try:
        return int(blob[index])
    except Exception:
        return int(default_value)


def blob_to_candidate(blob):
    """把 MaixPy blob 对象转换成核心逻辑可测试的数据结构。

    参数：
    - blob：`img.find_blobs(...)` 返回的目标对象。

    返回值：`BlobCandidate`，只包含追踪和画框需要的基础字段。
    副作用：无。角度字段不存在时填 0，保证后续协议仍可稳定输出。
    """
    return BlobCandidate(
        x=blob_value(blob, "x", 0),
        y=blob_value(blob, "y", 1),
        w=blob_value(blob, "w", 2),
        h=blob_value(blob, "h", 3),
        pixels=blob_value(blob, "pixels", 4),
        cx=blob_value(blob, "cx", 5),
        cy=blob_value(blob, "cy", 6),
        angle_deg=blob_value(blob, "rotation_deg", 7),
    )


def smooth_target_for_control(target, smooth_state, pickup_u, pickup_v):
    """把当前识别目标转换成更适合机械臂控制的平滑目标。

    参数：
    - target：`find_target()` 找到的原始目标；传入 None 时表示本帧未识别到目标；
    - smooth_state：`TargetSmoothState` 实例，用于保存跨帧平滑结果；
    - pickup_u/pickup_v：抓取参考点像素坐标。

    返回值：
    - 识别到目标，或处于丢失保持窗口内时返回 `VisionTarget`，其中 `cx/cy/dx/dy/area/score` 已经平滑；
    - 已经超出丢失保持窗口时返回 None，调用方应据此清零下游控制量。

    设计原因：
    - 原始 `find_blobs()` 每帧会有少量像素级跳动，直接发给机械臂会造成舵机抖动；
    - 静止时 alpha 设到 0.20，让 ±2 像素级别的底层噪声基本不会传到 ESP32；
    - 急动时 alpha 设到 0.85，让大幅运动不至于被平滑硬拖在后面；
    - 丢失 1~3 帧时仍按上次平滑结果继续输出，避免单帧误判让机械臂瞬间归零；
    - 使用整数指数平滑，避免在 MaixCAM2 上引入额外浮点开销。

    副作用：
    - 会原地更新 `smooth_state` 的所有字段，包括锁定计数和丢失计数；
    - 只读取 pickup_u/pickup_v，不会修改抓取参考点。
    """
    if target is None:
        # 本帧未识别到目标：在丢失保持窗口内仍输出最后一次平滑结果；超出后才真正放手。
        smooth_state.locked_streak = 0
        smooth_state.lost_streak += 1
        if smooth_state.has_target and (smooth_state.lost_streak <= TARGET_LOST_HOLD_FRAMES):
            return VisionTarget(
                x=max(0, smooth_state.cx - smooth_state.last_w // 2),
                y=max(0, smooth_state.cy - smooth_state.last_h // 2),
                w=smooth_state.last_w,
                h=smooth_state.last_h,
                cx=smooth_state.cx,
                cy=smooth_state.cy,
                dx_px=smooth_state.dx_px,
                dy_px=smooth_state.dy_px,
                angle_deg=0,
                area=smooth_state.area,
                score=0,
            )

        smooth_state.has_target = False
        smooth_state.lost_streak = 0
        return None

    smooth_state.lost_streak = 0
    if smooth_state.locked_streak < 0xFFFF:
        smooth_state.locked_streak += 1

    smooth_state.last_w = max(1, int(target.w))
    smooth_state.last_h = max(1, int(target.h))

    if not smooth_state.has_target:
        smooth_state.has_target = True
        smooth_state.cx = int(target.cx)
        smooth_state.cy = int(target.cy)
        smooth_state.area = int(target.area)
    else:
        move_px = max(
            abs(int(target.cx) - smooth_state.cx),
            abs(int(target.cy) - smooth_state.cy),
        )
        if move_px <= TARGET_MOTION_STILL_PX:
            alpha_num = TARGET_SMOOTH_ALPHA_STILL_NUM
        elif move_px <= TARGET_MOTION_SLOW_PX:
            alpha_num = TARGET_SMOOTH_ALPHA_SLOW_NUM
        else:
            alpha_num = TARGET_SMOOTH_ALPHA_FAST_NUM

        old_weight = TARGET_SMOOTH_ALPHA_DEN - alpha_num
        smooth_state.cx = int(
            (smooth_state.cx * old_weight + int(target.cx) * alpha_num)
            / TARGET_SMOOTH_ALPHA_DEN
        )
        smooth_state.cy = int(
            (smooth_state.cy * old_weight + int(target.cy) * alpha_num)
            / TARGET_SMOOTH_ALPHA_DEN
        )
        smooth_state.area = int(
            (smooth_state.area * old_weight + int(target.area) * alpha_num)
            / TARGET_SMOOTH_ALPHA_DEN
        )

    smooth_state.dx_px = int(smooth_state.cx - pickup_u)
    smooth_state.dy_px = int(smooth_state.cy - pickup_v)

    return type(target)(
        x=target.x,
        y=target.y,
        w=target.w,
        h=target.h,
        cx=smooth_state.cx,
        cy=smooth_state.cy,
        dx_px=smooth_state.dx_px,
        dy_px=smooth_state.dy_px,
        angle_deg=target.angle_deg,
        area=smooth_state.area,
        score=target.score,
    )


def compute_tracking_roi(smooth_state):
    """根据上一帧锁定结果计算下一帧的 ROI。

    参数：
    - smooth_state：当前的 `TargetSmoothState`，用于读取上一帧目标位置。

    返回值：
    - 已锁定且未丢失时返回 `(x, y, w, h)`，限制在图像边界内；
    - 否则返回 None，表示本帧需要全图搜索。

    设计原因：
    - 锁定后只搜目标周围区域，可以有效屏蔽背景里突然出现的同色干扰；
    - 丢失保持期内不缩 ROI，给目标重新进入画面留出余量；
    - 半宽/半高写在常量里，便于现场根据目标尺寸调整。
    """
    if (not smooth_state.has_target) or (smooth_state.lost_streak > 0):
        return None

    half_px = TARGET_ROI_HALF_PX
    x0 = max(0, int(smooth_state.cx) - half_px)
    y0 = max(0, int(smooth_state.cy) - half_px)
    x1 = min(CAMERA_WIDTH, int(smooth_state.cx) + half_px)
    y1 = min(CAMERA_HEIGHT, int(smooth_state.cy) + half_px)
    if (x1 - x0 <= 0) or (y1 - y0 <= 0):
        return None
    return (x0, y0, x1 - x0, y1 - y0)


def call_find_blobs(img, threshold_tuple, roi):
    """调用 MaixPy `find_blobs` 并按当前固件能力降级参数。

    参数：
    - img：摄像头当前帧；
    - threshold_tuple：六通道 LAB 阈值元组；
    - roi：`(x, y, w, h)` ROI 元组；None 表示全图搜索。

    返回值：
    - 调用成功返回原始 blob 列表；
    - 调用失败或固件不支持时返回 `[]`，避免主循环崩溃。

    设计原因：
    - 不同 MaixPy 固件版本对 `merge/margin/roi` 参数支持不一致；
    - 用层层降级而不是直接 raise，确保现场某个固件参数缺失时仍能识别出目标。

    副作用：无。该函数只做调用兼容封装。
    """
    base_kwargs = {
        "pixels_threshold": MIN_PIXELS,
        "area_threshold": MIN_PIXELS,
        "merge": True,
        "margin": 8,
    }

    try:
        if roi is not None:
            try:
                return img.find_blobs([threshold_tuple], roi=roi, **base_kwargs)
            except TypeError:
                # 当前固件 find_blobs 不支持 roi 参数，退回全图搜索。
                pass
        return img.find_blobs([threshold_tuple], **base_kwargs)
    except TypeError:
        # 退回最基础参数集合，兼容旧固件。
        try:
            return img.find_blobs(
                [threshold_tuple],
                pixels_threshold=MIN_PIXELS,
                area_threshold=MIN_PIXELS,
            )
        except Exception as exc:
            print("[VISION] find_blobs fallback failed:", exc)
            return []
    except Exception as exc:
        print("[VISION] find_blobs failed:", exc)
        return []


def find_target(img, config, pickup_u, pickup_v, smooth_state):
    """使用当前阈值在图像中寻找可追踪色块。

    参数：
    - img：摄像头拍到的原始图像；
    - config：LAB 阈值配置；
    - pickup_u/pickup_v：抓取参考点像素坐标；
    - smooth_state：上一帧平滑状态，用于跨帧选目标和 ROI 跟踪。

    返回值：
    - 找到目标时返回 `VisionTarget`；
    - 未找到目标或 MaixPy 查找失败时返回 None。

    主要流程：
    1. 已锁定时优先用 ROI 限制搜索区域，减少背景干扰；
    2. 通过兼容封装调用 `find_blobs`，对不同固件版本降级参数；
    3. 把候选色块交给跨帧连续性筛选，避免被瞬间干扰抢走。

    副作用：无。图像绘制和 I2C 输出由调用者完成。
    """
    roi = compute_tracking_roi(smooth_state)
    raw_blobs = call_find_blobs(img, config.as_tuple(), roi)

    candidates = [blob_to_candidate(raw_blob) for raw_blob in raw_blobs]

    prev_cx = int(smooth_state.cx) if smooth_state.has_target else None
    prev_cy = int(smooth_state.cy) if smooth_state.has_target else None

    return pick_best_blob_with_continuity(
        candidates,
        min_pixels=MIN_PIXELS,
        pickup_u=pickup_u,
        pickup_v=pickup_v,
        max_offset_px=MAX_OFFSET_PX,
        prev_cx=prev_cx,
        prev_cy=prev_cy,
    )


def build_binary_view(img, config):
    """生成调参页使用的二值化画面。

    参数：
    - img：摄像头原始图像；
    - config：当前 LAB 阈值。

    返回值：
    - 成功时返回二值化后的图像；
    - MaixPy `binary` 调用失败时返回原图，保证 UI 仍可操作。

    副作用：可能创建图像副本。基础版本优先保证调参直观性，后续再根据帧率优化。
    """
    try:
        return img.binary([config.as_tuple()], copy=True)
    except Exception as exc:
        print("[VISION] binary view failed:", exc)
        return img


def draw_tune_overlay(img, config, selected_field):
    """绘制二值化调参页的按钮和阈值状态。

    参数：
    - img：已经二值化的画面；
    - config：当前 LAB 阈值；
    - selected_field：当前被 `+/-` 操作的字段名。

    返回值：无。
    副作用：会在画面底部和右侧覆盖 UI 控件。
    """
    draw_text(img, 4, 8, "TUNE", color=COLOR_YELLOW, scale=1.0)
    draw_text(img, 4, 34, "{}={}".format(selected_field, getattr(config, selected_field)), color=COLOR_YELLOW, scale=0.85)
    draw_button(img, TRACK_BUTTON, "TRACK", selected=False)
    draw_button(img, PLUS_BUTTON, "+", selected=False)
    draw_button(img, MINUS_BUTTON, "-", selected=False)

    for index, field_name in enumerate(THRESHOLD_FIELDS):
        label = "{}\n{}".format(field_name.replace("_", ""), getattr(config, field_name))
        draw_button(img, get_field_button_rect(index), label, selected=(field_name == selected_field))


def draw_track_overlay(
    img,
    target,
    pickup_u,
    pickup_v,
    link_enabled,
    i2c_status_text,
    i2c_status_color,
    i2c_scan_text,
    i2c_scan_color,
    i2c_detail_text,
    i2c_detail_color,
):
    """绘制追踪抓取页的目标框、抓取参考点和当前偏差。

    参数：
    - img：摄像头原始画面；
    - target：当前筛选出的目标，None 表示未找到；
    - pickup_u/pickup_v：抓取参考点像素坐标。
    - link_enabled：当前是否允许后台线程尝试 I2C 链路。
    - i2c_status_text / i2c_status_color：追踪页显示的 I2C 写入状态；
    - i2c_scan_text / i2c_scan_color：追踪页显示的 I2C 总线扫描状态；
    - i2c_detail_text / i2c_detail_color：追踪页显示的 I2C 失败细节。

    返回值：无。
    副作用：会在画面上叠加追踪辅助信息，不改变识别结果。
    """
    draw_button(img, TUNE_BUTTON, "TUNE", selected=False)
    draw_button(img, LINK_BUTTON, "LINK\n{}".format("ON" if link_enabled else "OFF"), selected=link_enabled)
    img.draw_cross(pickup_u, pickup_v, color=COLOR_BLUE, size=10, thickness=2)
    draw_text(img, 86, 28, i2c_status_text, color=i2c_status_color, scale=0.8)
    draw_text(img, 86, 46, i2c_scan_text, color=i2c_scan_color, scale=0.8)
    if i2c_detail_text:
        draw_text(img, 86, 64, i2c_detail_text, color=i2c_detail_color, scale=0.8)

    if target is None:
        draw_text(img, 86, 10, "NO TARGET", color=COLOR_RED, scale=1.0)
        return

    img.draw_rect(target.x, target.y, target.w, target.h, color=COLOR_GREEN, thickness=2)
    img.draw_cross(target.cx, target.cy, color=COLOR_GREEN, size=8, thickness=2)
    img.draw_line(pickup_u, pickup_v, target.cx, target.cy, color=COLOR_YELLOW, thickness=2)
    draw_text(
        img,
        70,
        10,
        "dx={} dy={} area={}".format(target.dx_px, target.dy_px, target.area),
        color=COLOR_YELLOW,
        scale=0.8,
    )


def handle_tune_touch(x_pos, y_pos, config, selected_field):
    """处理调参页触摸事件。

    参数：
    - x_pos/y_pos：触摸释放时的坐标；
    - config：当前阈值配置；
    - selected_field：触摸前正在编辑的字段。

    返回值：
    - `(next_mode, next_selected_field)`；
    - `next_mode` 为 None 时表示继续留在当前页面。

    副作用：点 `+/-` 会原地修改 `config`；点 `TRACK` 会保存配置并切换页面。
    """
    if point_in_rect(x_pos, y_pos, TRACK_BUTTON):
        save_threshold_config(config)
        return MODE_TRACK, selected_field

    for index, field_name in enumerate(THRESHOLD_FIELDS):
        if point_in_rect(x_pos, y_pos, get_field_button_rect(index)):
            return None, field_name

    return None, selected_field


def handle_track_touch(x_pos, y_pos):
    """处理追踪页触摸事件。

    参数：
    - x_pos/y_pos：触摸释放时的坐标。

    返回值：
    - 返回 `(next_mode, toggle_link)`；
    - 点中 `TUNE` 时 `next_mode=MODE_TUNE`；
    - 点中 `LINK` 时 `toggle_link=True`；
    - 其它位置返回 `(None, False)`。

    副作用：无。这里只返回用户意图，真正的链路开关由主循环决定。
    """
    if point_in_rect(x_pos, y_pos, TUNE_BUTTON):
        return MODE_TUNE, False
    if point_in_rect(x_pos, y_pos, LINK_BUTTON):
        return None, True
    return None, False


def get_tune_adjust_button(x_pos, y_pos):
    """判断当前触点是否压在调参页的 `+/-` 按钮上。

    参数：
    - x_pos/y_pos：已经映射到图像坐标系内的触摸点。

    返回值：
    - 命中加号按钮返回 `plus`；
    - 命中减号按钮返回 `minus`；
    - 其它区域返回 None。

    副作用：无。
    """
    if point_in_rect(x_pos, y_pos, PLUS_BUTTON):
        return "plus"
    if point_in_rect(x_pos, y_pos, MINUS_BUTTON):
        return "minus"
    return None


def map_touch_to_image_coords(disp, touch_x, touch_y):
    """把触摸屏坐标转换成当前按钮绘制所使用的图像坐标。

    参数：
    - disp：当前显示对象，用于读取真实屏幕尺寸；
    - touch_x/touch_y：触摸屏返回的屏幕坐标。

    返回值：
    - 返回映射后的 `(image_x, image_y)`。

    主要流程：
    1. 优先调用 MaixPy 官方 `image.resize_map_pos_reverse`；
    2. 若当前固件没有该接口，则退回到本地纯 Python 等价实现。

    副作用：无。
    """
    try:
        return image.resize_map_pos_reverse(
            CAMERA_WIDTH,
            CAMERA_HEIGHT,
            disp.width(),
            disp.height(),
            image.Fit.FIT_CONTAIN,
            touch_x,
            touch_y,
        )
    except Exception:
        return map_display_point_to_image(
            image_width=CAMERA_WIDTH,
            image_height=CAMERA_HEIGHT,
            display_width=disp.width(),
            display_height=disp.height(),
            display_x=touch_x,
            display_y=touch_y,
        )


def main():
    """MaixCAM2 主循环入口。

    参数：无。
    返回值：无。

    主要流程：
    1. 初始化摄像头、屏幕、触摸和可选 I2C 后台线程；
    2. TUNE 模式显示二值化画面并允许调 LAB 阈值；
    3. TRACK 模式查找最大有效色块，显示 `dx/dy/area` 并通过 I2C 输出结果；
    4. I2C 阻塞调用全部放到后台线程，避免通讯失败时拖慢显示主循环；
    5. 退出时由 MaixPy `app.need_exit()` 控制。

    副作用：会持续占用摄像头和屏幕；I2C 链路正常时 TRACK 模式每帧输出一条视觉结果。
    """
    cam = camera.Camera(CAMERA_WIDTH, CAMERA_HEIGHT)
    disp = display.Display()
    ts = touchscreen.TouchScreen()

    config = load_threshold_config()
    selected_field = THRESHOLD_FIELDS[0]
    current_mode = MODE_TUNE
    frame_id = 0
    pickup_u = CAMERA_WIDTH // 2
    pickup_v = CAMERA_HEIGHT // 2
    was_pressed = False
    last_touch = (0, 0)
    adjust_state = LongPressAdjustState()
    comm_state = I2CCommState(bus_available=False)
    i2c_worker = create_i2c_worker_or_none(comm_state)
    previous_mode = current_mode
    target_smooth_state = TargetSmoothState()

    while not app.need_exit():
        frame = cam.read()
        now_ms = time.ticks_ms()
        touch_x, touch_y, is_pressed = ts.read()

        if is_pressed:
            mapped_x, mapped_y = map_touch_to_image_coords(disp, touch_x, touch_y)
            was_pressed = True
            last_touch = (touch_x, touch_y)

            if current_mode == MODE_TUNE:
                active_button = get_tune_adjust_button(mapped_x, mapped_y)
                step_count = consume_long_press_adjust_step(
                    adjust_state,
                    active_button,
                    now_ms,
                    LONG_PRESS_REPEAT_MS,
                )
                if step_count > 0:
                    if active_button == "plus":
                        config.adjust_channel(selected_field, THRESHOLD_STEP * step_count)
                    elif active_button == "minus":
                        config.adjust_channel(selected_field, -THRESHOLD_STEP * step_count)
            else:
                consume_long_press_adjust_step(adjust_state, None, now_ms, LONG_PRESS_REPEAT_MS)
        elif was_pressed:
            was_pressed = False
            consume_long_press_adjust_step(adjust_state, None, now_ms, LONG_PRESS_REPEAT_MS)
            release_x, release_y = map_touch_to_image_coords(disp, last_touch[0], last_touch[1])
            if current_mode == MODE_TUNE:
                next_mode, selected_field = handle_tune_touch(release_x, release_y, config, selected_field)
                if next_mode is not None:
                    current_mode = next_mode
            else:
                next_mode, toggle_link = handle_track_touch(release_x, release_y)
                if toggle_link:
                    current_link_enabled = (i2c_worker is not None) and i2c_worker.link_enabled
                    set_i2c_worker_link_enabled(i2c_worker, comm_state, now_ms, not current_link_enabled)
                if next_mode is not None:
                    current_mode = next_mode
        else:
            consume_long_press_adjust_step(adjust_state, None, now_ms, LONG_PRESS_REPEAT_MS)

        if previous_mode != current_mode:
            if current_mode == MODE_TRACK:
                set_i2c_worker_track_mode(i2c_worker, comm_state, now_ms, True)
            else:
                set_i2c_worker_track_mode(i2c_worker, comm_state, now_ms, False)
            previous_mode = current_mode

        if current_mode == MODE_TUNE:
            show_img = build_binary_view(frame, config)
            draw_tune_overlay(show_img, config, selected_field)
        else:
            raw_target = find_target(frame, config, pickup_u, pickup_v, target_smooth_state)
            target = smooth_target_for_control(raw_target, target_smooth_state, pickup_u, pickup_v)
            frame_id = (frame_id + 1) & 0xFFFF

            # 只有连续看到目标超过锁定阈值，才把 found 标记为 True 让 ESP32 真的去动；
            # 否则即便平滑层缓存到了目标，也按 "found=False" 上报，避免单帧误识别让机械臂瞬间转向。
            target_locked = (target is not None) and (
                target_smooth_state.locked_streak >= TARGET_LOCK_FRAMES
            )

            if (target is None) or (not target_locked):
                result_frame = pack_i2c_result_frame(frame_id, False, 0, 0, 0, 0, 0)
            else:
                result_frame = pack_i2c_result_frame(
                    frame_id,
                    True,
                    target.dx_px,
                    target.dy_px,
                    target.angle_deg,
                    target.score,
                    target.area,
                )
            # 主线程不再直接触碰 I2C，只把最新视觉结果交给后台线程异步处理。
            submit_i2c_result_frame(i2c_worker, result_frame)
            i2c_status_text, i2c_status_color = get_i2c_status_display(comm_state, now_ms)
            i2c_scan_text, i2c_scan_color = get_i2c_scan_display(comm_state)
            i2c_detail_text, i2c_detail_color = get_i2c_detail_display(comm_state)
            show_img = frame
            draw_track_overlay(
                show_img,
                target,
                pickup_u,
                pickup_v,
                comm_state.link_enabled,
                i2c_status_text,
                i2c_status_color,
                i2c_scan_text,
                i2c_scan_color,
                i2c_detail_text,
                i2c_detail_color,
            )

        disp.show(show_img, fit=image.Fit.FIT_CONTAIN)
        time.sleep_ms(10)

    if i2c_worker is not None:
        i2c_worker.running = False


if __name__ == "__main__":
    main()
