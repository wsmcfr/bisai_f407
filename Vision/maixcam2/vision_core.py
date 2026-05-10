"""MaixCAM2 颜色阈值、目标选择和结果协议的纯 Python 核心逻辑。

本文件不直接导入 MaixPy 的 camera/display/image 模块，目的是让阈值边界、
目标筛选和输出协议可以在普通 Python 环境下用单元测试验证。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, Optional, Tuple


# LAB 阈值字段顺序与 MaixPy/OpenMV 风格的 find_blobs 阈值元组保持一致。
THRESHOLD_FIELDS = ("l_min", "l_max", "a_min", "a_max", "b_min", "b_max")

# MaixPy find_blobs 使用 LAB 空间阈值；L 通道通常为 0-100，A/B 通道为 -128-127。
LAB_LIMITS = {
    "l": (0, 100),
    "a": (-128, 127),
    "b": (-128, 127),
}

# MaixCAM2 与 ESP32 机械臂控制板之间的 I2C 从机地址。
I2C_SLAVE_ADDR = 0x42

# 固定长度视觉结果帧：2 字节帧头 + 1 字节版本 + 1 字节标志 +
# 2 字节 frame_id + 2 字节 dx + 2 字节 dy + 2 字节 angle +
# 1 字节 score + 2 字节 area + 1 字节 checksum。
I2C_FRAME_LENGTH = 16


def clamp_threshold_value(value: int, lower: int, upper: int) -> int:
    """把阈值限制在指定闭区间内。

    参数：
    - value：准备写入阈值配置的原始值；
    - lower：当前通道允许的最小值；
    - upper：当前通道允许的最大值。

    返回值：
    - 返回已经限制到 `[lower, upper]` 范围内的整数。

    副作用：无。该函数只做纯计算，便于在 MaixCAM2 真机外测试。
    """
    if value < lower:
        return lower
    if value > upper:
        return upper
    return value


def map_display_point_to_image(
    image_width: int,
    image_height: int,
    display_width: int,
    display_height: int,
    display_x: int,
    display_y: int,
) -> Tuple[int, int]:
    """把屏幕坐标映射回 `FIT_CONTAIN` 图像坐标。

    参数：
    - image_width/image_height：原始图像尺寸；
    - display_width/display_height：屏幕或显示目标尺寸；
    - display_x/display_y：触摸屏返回的显示坐标。

    返回值：
    - 返回映射后的图像坐标 `(image_x, image_y)`。

    主要流程：
    1. 依据 `FIT_CONTAIN` 规则计算缩放后的图像尺寸；
    2. 扣除黑边偏移量；
    3. 再按缩放比例换算回原始图像坐标。

    副作用：无。

    说明：
    - 这个公式与 MaixPy 官方触摸屏文档里对 `image.resize_map_pos_reverse`
      的解释保持一致，便于在普通 Python 环境下做单元测试。
    """
    if (image_width <= 0) or (image_height <= 0) or (display_width <= 0) or (display_height <= 0):
        raise ValueError("image/display size must be positive")

    scale = min(float(display_width) / float(image_width), float(display_height) / float(image_height))
    mapped_width = float(image_width) * scale
    mapped_height = float(image_height) * scale
    offset_x = (float(display_width) - mapped_width) / 2.0
    offset_y = (float(display_height) - mapped_height) / 2.0

    image_x = int((float(display_x) - offset_x) / scale)
    image_y = int((float(display_y) - offset_y) / scale)
    return image_x, image_y


@dataclass
class ThresholdConfig:
    """颜色追踪使用的 LAB 六通道阈值配置。"""

    l_min: int = 55   # L 通道下限，默认采用更宽松、通用性更强的黄色阈值范围。
    l_max: int = 100  # L 通道上限，默认允许较亮黄色目标，适配常见室内灯光。
    a_min: int = -80  # A 通道下限，默认放宽到偏绿色一侧，提升黄色兼容范围。
    a_max: int = 20   # A 通道上限，默认覆盖常见黄色在 A 通道上的波动。
    b_min: int = 40   # B 通道下限，默认保证足够的黄蓝分离度。
    b_max: int = 127  # B 通道上限，默认覆盖饱和度更高的亮黄色目标。

    def adjust_channel(self, field_name: str, delta: int) -> None:
        """按增量调节一个阈值字段并保持上下限关系合法。

        参数：
        - field_name：必须是 `l_min/l_max/a_min/a_max/b_min/b_max` 之一；
        - delta：本次触摸按钮产生的调节增量，可正可负。

        返回值：无。

        主要流程：
        1. 根据字段名前缀找到 LAB 通道的合法范围；
        2. 把新值夹紧到该通道范围内；
        3. 如果调节后下限大于上限，就把当前字段压到另一侧边界。

        副作用：会修改当前配置对象的对应字段。
        """
        if field_name not in THRESHOLD_FIELDS:
            raise ValueError("unsupported threshold field: {}".format(field_name))

        channel_name = field_name[0]
        lower, upper = LAB_LIMITS[channel_name]
        current_value = getattr(self, field_name)
        next_value = clamp_threshold_value(current_value + delta, lower, upper)
        setattr(self, field_name, next_value)
        self._normalize_channel_pair(channel_name, field_name)

    def as_tuple(self) -> Tuple[int, int, int, int, int, int]:
        """生成 MaixPy `find_blobs`/`binary` 可直接使用的 LAB 阈值元组。

        参数：无。
        返回值：按 `(l_min, l_max, a_min, a_max, b_min, b_max)` 排列的元组。
        副作用：无。
        """
        return (self.l_min, self.l_max, self.a_min, self.a_max, self.b_min, self.b_max)

    def to_dict(self) -> Dict[str, int]:
        """把阈值配置转换为 JSON 可保存的字典。

        参数：无。
        返回值：字段名到整数阈值的映射。
        副作用：无。
        """
        return {field_name: int(getattr(self, field_name)) for field_name in THRESHOLD_FIELDS}

    @classmethod
    def from_dict(cls, raw_config: Dict[str, int]) -> "ThresholdConfig":
        """从字典恢复阈值配置并修正越界值。

        参数：
        - raw_config：从 JSON 或其它配置来源读取到的字段映射。

        返回值：
        - 返回一个已经完成范围限制和上下限归一化的 `ThresholdConfig`。

        副作用：无。非法或缺失字段会回退到默认值，避免坏配置导致启动失败。
        """
        config = cls()
        for field_name in THRESHOLD_FIELDS:
            if field_name in raw_config:
                try:
                    setattr(config, field_name, int(raw_config[field_name]))
                except (TypeError, ValueError):
                    pass
        config.clamp_all()
        return config

    def clamp_all(self) -> None:
        """修正所有通道的越界值和上下限关系。

        参数：无。
        返回值：无。
        副作用：会原地更新当前对象，适合配置文件加载后立即调用。
        """
        for channel_name in ("l", "a", "b"):
            lower, upper = LAB_LIMITS[channel_name]
            min_field = "{}_min".format(channel_name)
            max_field = "{}_max".format(channel_name)
            setattr(self, min_field, clamp_threshold_value(getattr(self, min_field), lower, upper))
            setattr(self, max_field, clamp_threshold_value(getattr(self, max_field), lower, upper))
            self._normalize_channel_pair(channel_name, max_field)

    def _normalize_channel_pair(self, channel_name: str, changed_field: str) -> None:
        """保证同一 LAB 通道的下限不大于上限。

        参数：
        - channel_name：`l/a/b` 之一，用来定位字段对；
        - changed_field：刚被用户调节的字段，用来决定压哪一侧。

        返回值：无。
        副作用：当上下限交叉时，会把当前字段压到另一侧边界。
        """
        min_field = "{}_min".format(channel_name)
        max_field = "{}_max".format(channel_name)
        min_value = getattr(self, min_field)
        max_value = getattr(self, max_field)

        if min_value <= max_value:
            return

        if changed_field == min_field:
            setattr(self, min_field, max_value)
        else:
            setattr(self, max_field, min_value)


@dataclass
class LongPressAdjustState:
    """记录 `+/-` 长按连发状态。"""

    active_button: str | None = None  # 当前正在长按的按钮名称，允许值为 `plus`、`minus` 或 None。
    last_emit_ms: int | None = None   # 最近一次已经实际发出加减操作的时刻，单位毫秒，基于 `ticks_ms()`。


def consume_long_press_adjust_step(
    state: LongPressAdjustState,
    active_button: str | None,
    now_ms: int,
    repeat_ms: int = 100,
) -> int:
    """根据当前长按状态计算本轮应该追加多少次加减操作。

    参数：
    - state：长按连发状态对象，不能为空；
    - active_button：当前触点是否压在 `plus/minus` 按钮上，不在按钮上时传 None；
    - now_ms：当前单调递增毫秒计时，建议使用 MaixPy `time.ticks_ms()`；
    - repeat_ms：连续触发周期，默认 100ms。

    返回值：
    - 返回本轮应执行的步数；
    - 首次按下按钮立即返回 1；
    - 按住期间每跨过一个 `repeat_ms` 周期再返回 1；
    - 若主循环偶发卡顿跨过多个周期，则返回累计步数，避免实际连发速度变慢。

    副作用：
    - 会更新 `state.active_button` 和 `state.last_emit_ms`；
    - 当 `active_button` 为 None 时会清空状态，表示已经松手或移出按钮区域。
    """
    if repeat_ms <= 0:
        raise ValueError("repeat_ms must be positive")

    if active_button is None:
        state.active_button = None
        state.last_emit_ms = None
        return 0

    if active_button not in ("plus", "minus"):
        raise ValueError("unsupported active_button: {}".format(active_button))

    if (state.active_button != active_button) or (state.last_emit_ms is None):
        state.active_button = active_button
        state.last_emit_ms = int(now_ms)
        return 1

    elapsed_ms = int(now_ms) - state.last_emit_ms
    if elapsed_ms < repeat_ms:
        return 0

    step_count = elapsed_ms // repeat_ms
    state.last_emit_ms += step_count * repeat_ms
    return int(step_count)


@dataclass(frozen=True)
class BlobCandidate:
    """从 MaixPy blob 对象抽象出的候选目标字段。"""

    x: int          # 目标外接矩形左上角 X 坐标，单位像素。
    y: int          # 目标外接矩形左上角 Y 坐标，单位像素。
    w: int          # 目标外接矩形宽度，单位像素。
    h: int          # 目标外接矩形高度，单位像素。
    pixels: int     # 二值区域包含的像素数量，用于过滤噪声和选择最大目标。
    cx: int         # 目标中心 X 坐标，单位像素。
    cy: int         # 目标中心 Y 坐标，单位像素。
    angle_deg: int  # 目标旋转角度，单位度；无角度信息时填 0。


@dataclass(frozen=True)
class VisionTarget:
    """筛选后的视觉目标结果。"""

    x: int          # 目标外接矩形左上角 X 坐标，单位像素。
    y: int          # 目标外接矩形左上角 Y 坐标，单位像素。
    w: int          # 目标外接矩形宽度，单位像素。
    h: int          # 目标外接矩形高度，单位像素。
    cx: int         # 目标中心 X 坐标，单位像素。
    cy: int         # 目标中心 Y 坐标，单位像素。
    dx_px: int      # 目标中心相对抓取参考点的 X 偏差，右侧为正，单位像素。
    dy_px: int      # 目标中心相对抓取参考点的 Y 偏差，下侧为正，单位像素。
    angle_deg: int  # 目标旋转角度，单位度。
    area: int       # 目标像素面积，用于置信度和误识别过滤。
    score: int      # 0-100 的粗略置信度，当前由面积映射得到。


def pick_largest_valid_blob(
    blobs: Iterable[BlobCandidate],
    min_pixels: int,
    pickup_u: int,
    pickup_v: int,
    max_offset_px: int,
) -> Optional[VisionTarget]:
    """从候选色块中选出可抓取的最大目标。

    参数：
    - blobs：MaixPy `find_blobs` 结果转换出的候选目标序列；
    - min_pixels：最小有效像素面积，小于该值视作噪声；
    - pickup_u：抓取参考点 X 坐标，手持测试时默认可用图像中心；
    - pickup_v：抓取参考点 Y 坐标，手持测试时默认可用图像中心；
    - max_offset_px：允许上报的最大偏差，超过该值说明目标离抓取区域太远。

    返回值：
    - 找到有效目标时返回 `VisionTarget`；
    - 没有满足面积和偏差要求的目标时返回 `None`。

    副作用：无。真正的画框、I2C 发送和机械臂动作由上层处理。
    """
    best_blob = None

    for blob in blobs:
        if blob.pixels < min_pixels:
            continue

        dx_px = blob.cx - pickup_u
        dy_px = blob.cy - pickup_v
        if (abs(dx_px) > max_offset_px) or (abs(dy_px) > max_offset_px):
            continue

        if (best_blob is None) or (blob.pixels > best_blob.pixels):
            best_blob = blob

    if best_blob is None:
        return None

    dx_px = best_blob.cx - pickup_u
    dy_px = best_blob.cy - pickup_v
    return VisionTarget(
        x=best_blob.x,
        y=best_blob.y,
        w=best_blob.w,
        h=best_blob.h,
        cx=best_blob.cx,
        cy=best_blob.cy,
        dx_px=dx_px,
        dy_px=dy_px,
        angle_deg=best_blob.angle_deg,
        area=best_blob.pixels,
        score=score_from_area(best_blob.pixels, min_pixels),
    )


def score_from_area(area: int, min_pixels: int) -> int:
    """根据目标面积生成 0-100 的粗略置信度。

    参数：
    - area：当前目标像素面积；
    - min_pixels：最小有效面积阈值。

    返回值：
    - 面积小于等于 0 时返回 0；
    - 面积达到 `min_pixels * 4` 时封顶 100。

    副作用：无。该分数只是给 ESP32/调试日志排序参考，不代表模型概率。
    """
    if (area <= 0) or (min_pixels <= 0):
        return 0

    scaled_score = int((area * 100) / (min_pixels * 4))
    return clamp_threshold_value(scaled_score, 0, 100)


def _pack_signed_int16(value: int) -> Tuple[int, int]:
    """把有符号 16 位整数打包成小端双字节。

    参数：
    - value：待打包的整数。

    返回值：
    - 返回 `(low_byte, high_byte)`。

    副作用：无。
    """
    packed_value = int(value) & 0xFFFF
    return packed_value & 0xFF, (packed_value >> 8) & 0xFF


def _unpack_signed_int16(low_byte: int, high_byte: int) -> int:
    """把两个小端字节恢复成有符号 16 位整数。

    参数：
    - low_byte/high_byte：小端顺序的低字节和高字节。

    返回值：
    - 返回 `-32768 ~ 32767` 范围内的整数。

    副作用：无。
    """
    raw_value = (int(high_byte) << 8) | int(low_byte)
    if raw_value & 0x8000:
        return raw_value - 0x10000
    return raw_value


def pack_i2c_result_frame(
    frame_id: int,
    found: bool,
    dx_px: int,
    dy_px: int,
    angle_deg: int,
    score: int,
    area: int,
) -> bytes:
    """把视觉结果打包成固定长度 I2C 二进制帧。

    参数：
    - frame_id：16 位递增帧号；
    - found：是否找到目标；
    - dx_px/dy_px：相对抓取参考点的像素偏差；
    - angle_deg：目标角度；
    - score：0-100 的粗略置信度；
    - area：目标像素面积。

    返回值：
    - 长度固定为 `I2C_FRAME_LENGTH` 的 `bytes` 对象。

    帧格式：
    - Byte0-1：固定帧头 `0xA5 0x5A`
    - Byte2：协议版本，当前固定 `0x01`
    - Byte3：标志位，bit0=found
    - Byte4-5：frame_id，小端
    - Byte6-7：dx_px，小端有符号 16 位
    - Byte8-9：dy_px，小端有符号 16 位
    - Byte10-11：angle_deg，小端有符号 16 位
    - Byte12：score
    - Byte13-14：area，小端无符号 16 位
    - Byte15：checksum，等于前 15 字节求和后取低 8 位

    副作用：无。
    """
    frame_bytes = [0xA5, 0x5A, 0x01, 0x01 if found else 0x00]
    frame_bytes.extend(_pack_signed_int16(frame_id))
    frame_bytes.extend(_pack_signed_int16(dx_px))
    frame_bytes.extend(_pack_signed_int16(dy_px))
    frame_bytes.extend(_pack_signed_int16(angle_deg))
    frame_bytes.append(clamp_threshold_value(int(score), 0, 100))
    area_value = max(0, min(int(area), 0xFFFF))
    frame_bytes.append(area_value & 0xFF)
    frame_bytes.append((area_value >> 8) & 0xFF)
    frame_bytes.append(sum(frame_bytes) & 0xFF)
    return bytes(frame_bytes)


def parse_i2c_result_frame(frame_bytes: bytes) -> Dict[str, int | bool]:
    """解析固定长度 I2C 视觉结果帧。

    参数：
    - frame_bytes：长度固定为 `I2C_FRAME_LENGTH` 的二进制帧。

    返回值：
    - 返回字段字典，供单元测试、调试脚本或 PC 工具复用。

    副作用：无。遇到非法帧会抛出 `ValueError`。
    """
    if len(frame_bytes) != I2C_FRAME_LENGTH:
        raise ValueError("invalid frame length: {}".format(len(frame_bytes)))
    if (frame_bytes[0] != 0xA5) or (frame_bytes[1] != 0x5A):
        raise ValueError("invalid frame header")
    if frame_bytes[2] != 0x01:
        raise ValueError("unsupported frame version: {}".format(frame_bytes[2]))
    if (sum(frame_bytes[:-1]) & 0xFF) != frame_bytes[-1]:
        raise ValueError("invalid checksum")

    area_value = frame_bytes[13] | (frame_bytes[14] << 8)
    return {
        "slave_addr": I2C_SLAVE_ADDR,
        "frame_id": _unpack_signed_int16(frame_bytes[4], frame_bytes[5]) & 0xFFFF,
        "found": (frame_bytes[3] & 0x01) != 0,
        "dx_px": _unpack_signed_int16(frame_bytes[6], frame_bytes[7]),
        "dy_px": _unpack_signed_int16(frame_bytes[8], frame_bytes[9]),
        "angle_deg": _unpack_signed_int16(frame_bytes[10], frame_bytes[11]),
        "score": int(frame_bytes[12]),
        "area": int(area_value),
    }


def pack_result_line(
    frame_id: int,
    found: bool,
    dx_px: int,
    dy_px: int,
    angle_deg: int,
    score: int,
    area: int,
) -> str:
    """打包历史兼容用的视觉结果文本帧。

    参数：
    - frame_id：递增帧序号，用于 ESP32 判断数据是否新鲜；
    - found：是否找到目标；
    - dx_px/dy_px：相对抓取参考点的像素偏差；
    - angle_deg：目标角度；
    - score：0-100 的粗略置信度；
    - area：目标像素面积。

    返回值：
    - 以换行结尾的 CSV 文本帧，格式为
      `V1,frame_id,found,dx,dy,angle,score,area\n`。

    副作用：无。该函数仅保留给 PC 调试或历史串口桥接脚本复用，当前主链路已经切到 I2C 二进制帧。
    """
    found_value = 1 if found else 0
    return "V1,{},{},{},{},{},{},{}\n".format(
        int(frame_id) & 0xFFFF,
        found_value,
        int(dx_px),
        int(dy_px),
        int(angle_deg),
        clamp_threshold_value(int(score), 0, 100),
        max(0, int(area)),
    )
