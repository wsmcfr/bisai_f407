"""MaixCAM2 I2C 引脚与地址诊断程序。

运行方式：
1. 把本文件放到 MaixCAM2 的 `Vision/maixcam2` 目录；
2. 在 MaixPy 中运行 `i2c_probe.py`；
3. 观察屏幕上 `I2C6 A1/A0` 和 `I2C7 A8/A9` 哪一组能扫到 `0x42`。

这个程序只做诊断，不启动摄像头，不进入颜色追踪流程，避免把图像处理卡顿
和 I2C 电气层问题混在一起。
"""

TARGET_ADDR = 0x42
PROBE_VERSION = "2026-05-10-scan-safe-v3"
I2C_FREQ_HZ = 50000
SCREEN_WIDTH = 320
SCREEN_HEIGHT = 240
TEXT_SCALE = 0.72
LINE_HEIGHT = 16
RESCAN_DEBOUNCE_MS = 500
PROBE_FULL_SCAN_WHEN_TARGET_MISSING = True
PROBE_WRITE_WHEN_TARGET_MISSING = False
PROBE_READ_WHEN_TARGET_MISSING = False
PROBE_READ_LENGTH = 4


# 诊断候选项：
# - 当前已经实测 I2C6 A1/A0 能扫到 ESP32 `0x42`，所以默认只保留这一组；
# - I2C7 A8/A9 未接到当前机械臂 I2C 线，继续切换它会让部分 MaixPy 固件底层异常退出。
I2C_CANDIDATES = (
    {
        "name": "I2C6 A1/A0",
        "bus": 6,
        "scl_pin": "A1",
        "scl_func": "I2C6_SCL",
        "sda_pin": "A0",
        "sda_func": "I2C6_SDA",
    },
)


def clip_text(text, max_len):
    """把过长文本截断到屏幕可读长度。

    参数：
    - text：任意对象，会先转成字符串；
    - max_len：允许显示的最大字符数。

    返回值：
    - 不超过 `max_len` 的字符串。

    副作用：无。该函数只用于避免 pinmap 支持项过长时挤出屏幕。
    """
    text = str(text)
    if len(text) <= max_len:
        return text
    return text[: max_len - 3] + "..."


def normalize_i2c_addresses(raw_addresses):
    """把 MaixPy `scan()` 返回值整理成 7 位地址元组。

    参数：
    - raw_addresses：`i2c.scan()` 可能返回的列表、元组、None 或单个值。

    返回值：
    - 排序后的 7 位 I2C 地址元组，例如 `(0x42,)`。

    设计原因：
    - 不同 MaixPy 版本的 scan 返回值可能略有差异；
    - 诊断程序只关心 0x08~0x77 范围内的有效 7 位地址。
    """
    normalized = []

    if raw_addresses is None:
        raw_addresses = []
    elif isinstance(raw_addresses, int):
        raw_addresses = [raw_addresses]

    for raw_addr in raw_addresses:
        try:
            addr_value = int(raw_addr)
        except Exception:
            continue

        if (0x00 <= addr_value <= 0x7F) and (addr_value not in normalized):
            normalized.append(addr_value)

    return tuple(sorted(normalized))


def make_probe_frame():
    """生成一帧合法但表示“未发现目标”的视觉测试帧。

    参数：无。
    返回值：
    - 16 字节 `bytes`，格式与 `vision_core.pack_i2c_result_frame()` 保持一致。

    设计原因：
    - scan 只能证明从机地址应答；
    - 若 scan 已看到 0x42，再写一帧合法空目标帧，可以进一步证明
      MaixCAM2 主机写入和 ESP32 `onReceive` 链路都能跑通；
    - 空目标帧不会要求机械臂动作。
    """
    frame = bytearray(16)
    frame[0] = 0xA5
    frame[1] = 0x5A
    frame[2] = 0x01
    frame[3] = 0x00
    frame[15] = sum(frame[:15]) & 0xFF
    return bytes(frame)


def format_byte_list(data_bytes):
    """把读取到的字节格式化为十六进制短文本。

    参数：
    - data_bytes：`readfrom()` 返回的 bytes、bytearray 或可迭代整数。

    返回值：
    - 例如 `A5 42 01 E8`；无法解析或为空时返回 `none`。

    副作用：无。该函数只服务屏幕和串口诊断显示。
    """
    if not data_bytes:
        return "none"

    formatted = []
    try:
        for value in data_bytes:
            formatted.append("{:02X}".format(int(value) & 0xFF))
    except Exception:
        return clip_text(data_bytes, 28)

    return " ".join(formatted) if formatted else "none"


def get_pinmap_text(pinmap_module, pin_name):
    """读取某个 MaixCAM2 引脚当前支持的复用功能。

    参数：
    - pinmap_module：MaixPy `pinmap` 模块；
    - pin_name：例如 `A8`、`A9`、`A0`、`A1`。

    返回值：
    - 适合屏幕显示的短文本；读取失败时返回错误摘要。

    副作用：
    - 只读 pinmap 信息，不改变引脚功能。
    """
    try:
        return clip_text(pinmap_module.get_pin_functions(pin_name), 34)
    except Exception as exc:
        return "ERR {}".format(clip_text(exc, 28))


def read_gpio_idle_level(pinmap_module, gpio_module, err_module, pin_name):
    """读取某个候选 I2C 引脚的空闲电平。

    参数：
    - pinmap_module：MaixPy `pinmap` 模块；
    - gpio_module：MaixPy `gpio` 模块；
    - err_module：MaixPy `err` 模块，用于检查 pinmap 返回码；
    - pin_name：例如 `A1`、`A0`、`A8`、`A9`。

    返回值：
    - 成功时返回 0 或 1；
    - 失败时返回 `None`。

    设计原因：
    - I2C 总线空闲时 SCL/SDA 都应该被外部上拉为高电平；
    - 机械臂控制板原理图里 SDA/SCL 有 5.1k 上拉到 3.3V；
    - 如果这里读到 0 或无法稳定为 1，优先查 GND、线序、插头方向或短路。

    副作用：
    - 会临时把该引脚切到 GPIO 输入模式；
    - 后续 `probe_candidate()` 会重新切回 I2C 功能。
    """
    gpio_func_name = "GPIO{}".format(pin_name)

    try:
        err_module.check_raise(
            pinmap_module.set_pin_function(pin_name, gpio_func_name),
            "set {} failed".format(gpio_func_name),
        )
        gpio_obj = gpio_module.GPIO(gpio_func_name, gpio_module.Mode.IN, gpio_module.Pull.PULL_NONE)
        return int(gpio_obj.value())
    except Exception as exc:
        print("[I2C_PROBE] gpio idle read failed pin={} error={}".format(pin_name, clip_text(exc, 42)))
        return None


def read_candidate_idle_levels(pinmap_module, gpio_module, err_module, candidate):
    """读取一组 I2C 候选线的 SCL/SDA 空闲电平。

    参数：
    - pinmap_module/gpio_module/err_module：MaixPy 相关模块；
    - candidate：`I2C_CANDIDATES` 中的一项。

    返回值：
    - `(scl_level, sda_level)`，每项为 0、1 或 None。

    副作用：
    - 会临时把候选 SCL/SDA 引脚切到 GPIO 输入；
    - 调用者随后应重新设置为 I2C 功能再扫描。
    """
    scl_level = read_gpio_idle_level(pinmap_module, gpio_module, err_module, candidate["scl_pin"])
    sda_level = read_gpio_idle_level(pinmap_module, gpio_module, err_module, candidate["sda_pin"])
    return scl_level, sda_level


def scan_with_fallback(i2c_bus):
    """优先扫描目标地址，没命中时再做全总线扫描。

    参数：
    - i2c_bus：已经初始化好的 MaixPy I2C 对象。

    返回值：
    - `(addresses, mode)`，addresses 为规范化后的地址元组，mode 为扫描方式说明。

    设计原因：
    - MaixPy 新接口支持 `scan(addr)`，只测 0x42 时比全总线扫描更快；
    - 默认只扫 0x42，避免全总线扫描在接线异常时拖住诊断程序；
    - 如需判断“是否有其它地址”，可把 `PROBE_FULL_SCAN_WHEN_TARGET_MISSING` 改为 True；
    - 若当前固件不支持带参数 scan，则自动退回 `scan()`，兼容旧固件。
    """
    try:
        target_addresses = normalize_i2c_addresses(i2c_bus.scan(TARGET_ADDR))
        if TARGET_ADDR in target_addresses:
            return target_addresses, "target"

        if PROBE_FULL_SCAN_WHEN_TARGET_MISSING:
            full_addresses = normalize_i2c_addresses(i2c_bus.scan())
            return full_addresses, "target+full"

        return target_addresses, "target-only"
    except TypeError:
        return normalize_i2c_addresses(i2c_bus.scan()), "full"


def probe_candidate(pinmap_module, i2c_module, err_module, candidate):
    """测试一组 MaixCAM2 I2C 引脚配置。

    参数：
    - pinmap_module：MaixPy `pinmap` 模块；
    - i2c_module：MaixPy `i2c` 模块；
    - err_module：MaixPy `err` 模块，用于检查 pinmap 返回码；
    - candidate：`I2C_CANDIDATES` 中的一项。

    返回值：
    - 字典，包含 pinmap 设置、scan 地址、0x42 是否可见、写入是否成功等结果。

    副作用：
    - 会把候选引脚临时切换到对应 I2C 功能；
    - 若扫到 0x42，会写入一帧合法空目标帧用于确认从机接收路径。
    """
    result = {
        "name": candidate["name"],
        "ok": False,
        "visible": False,
        "addresses": (),
        "scan_mode": "",
        "write": "skip",
        "read": "skip",
        "error": "",
    }

    try:
        err_module.check_raise(
            pinmap_module.set_pin_function(candidate["scl_pin"], candidate["scl_func"]),
            "set {} failed".format(candidate["scl_func"]),
        )
        err_module.check_raise(
            pinmap_module.set_pin_function(candidate["sda_pin"], candidate["sda_func"]),
            "set {} failed".format(candidate["sda_func"]),
        )

        i2c_bus = i2c_module.I2C(candidate["bus"], i2c_module.Mode.MASTER, freq=I2C_FREQ_HZ)
        addresses, scan_mode = scan_with_fallback(i2c_bus)
        result["addresses"] = addresses
        result["scan_mode"] = scan_mode
        result["visible"] = TARGET_ADDR in addresses
        result["ok"] = True

        if result["visible"] or PROBE_WRITE_WHEN_TARGET_MISSING:
            # 即使 scan 没看到 0x42，也尝试写一次，避免误把 MaixPy scan 行为当成唯一证据。
            # 写入失败时不要让外层 except 覆盖扫描结果，而是把 NACK/异常直接显示出来。
            try:
                write_len = i2c_bus.writeto(TARGET_ADDR, make_probe_frame())
                # MaixPy `writeto()` 可能不抛异常，而是返回负错误码。
                # 负数同样表示底层写失败，不能显示成 OK，否则会误判 I2C 已经打通。
                if int(write_len) < 0:
                    result["write"] = "ERR {}".format(write_len)
                else:
                    result["write"] = "OK {}".format(write_len)
            except Exception as write_exc:
                result["write"] = "ERR {}".format(clip_text(write_exc, 38))
        elif not result["visible"]:
            result["write"] = "skip no-ack"

        if result["visible"] or PROBE_READ_WHEN_TARGET_MISSING:
            # ESP32 诊断固件补了 `onRequest`，所以这里额外读 4 字节。
            # 如果读成功，说明至少地址阶段和读请求回调已经进入 ESP32；
            # 如果读失败且 ESP32 端 REQ 计数不变，说明仍是 ACK/电气层问题。
            try:
                read_data = i2c_bus.readfrom(TARGET_ADDR, PROBE_READ_LENGTH)
                result["read"] = "OK {}".format(format_byte_list(read_data))
            except Exception as read_exc:
                result["read"] = "ERR {}".format(clip_text(read_exc, 38))
        elif not result["visible"]:
            result["read"] = "skip no-ack"
        return result
    except Exception as exc:
        result["error"] = clip_text(exc, 52)
        return result


def format_addr_list(addresses):
    """把地址列表格式化成屏幕显示文本。

    参数：
    - addresses：规范化后的 7 位地址元组。

    返回值：
    - 例如 `42 68`；没有地址时返回 `none`。

    副作用：无。
    """
    if not addresses:
        return "none"
    return " ".join("{:02X}".format(addr) for addr in addresses)


def run_probe():
    """执行一次完整 I2C 诊断。

    参数：无。
    返回值：
    - `(pin_lines, result_lines)`：
      - pin_lines：A8/A9/A0/A1 的 pinmap 支持项；
      - result_lines：每组 I2C 候选配置的 scan/write 结果。

    副作用：
    - 会访问 MaixCAM2 pinmap 和 I2C 外设；
    - 只在扫到目标地址时发送一帧空目标诊断帧。
    """
    from maix import err, gpio, i2c, pinmap

    pin_lines = []
    result_lines = []

    # 每次启动或点击重扫都打印版本号，防止 MaixCAM2 上仍在运行旧脚本而误判结果。
    print(
        "[I2C_PROBE] version={} freq={} target=0x{:02X} read_test={}".format(
            PROBE_VERSION,
            I2C_FREQ_HZ,
            TARGET_ADDR,
            PROBE_READ_WHEN_TARGET_MISSING,
        )
    )

    for pin_name in ("A8", "A9", "A0", "A1"):
        pin_lines.append("{}: {}".format(pin_name, get_pinmap_text(pinmap, pin_name)))

    for candidate in I2C_CANDIDATES:
        scl_idle, sda_idle = read_candidate_idle_levels(pinmap, gpio, err, candidate)
        result_lines.append("{} idle: SCL={} SDA={}".format(candidate["name"], scl_idle, sda_idle))
        print("[I2C_PROBE] {} idle scl={} sda={}".format(candidate["name"], scl_idle, sda_idle))

        result = probe_candidate(pinmap, i2c, err, candidate)
        if result["ok"]:
            status = "FOUND 42" if result["visible"] else "NO 42"
            # 串口同步打印诊断结果，便于用户把日志贴回来时直接判断是哪组引脚能看到 0x42。
            print(
                "[I2C_PROBE] {} status={} mode={} addrs={} write={}".format(
                    result["name"],
                    status,
                    result["scan_mode"],
                    format_addr_list(result["addresses"]),
                    result["write"],
                )
            )
            print("[I2C_PROBE] {} read={}".format(result["name"], result["read"]))
            result_lines.append(
                "{}: {} [{}] {}".format(
                    result["name"],
                    status,
                    result["scan_mode"],
                    format_addr_list(result["addresses"]),
                )
            )
            result_lines.append("  write: {}".format(result["write"]))
            result_lines.append("  read: {}".format(result["read"]))
        else:
            # 这里失败通常说明 pinmap 不支持该复用、I2C 外设初始化失败或底层驱动返回错误。
            print("[I2C_PROBE] {} status=ERR error={}".format(result["name"], result["error"]))
            result_lines.append("{}: ERR {}".format(result["name"], result["error"]))
            result_lines.append("  write: skip")
            result_lines.append("  read: skip")

    return pin_lines, result_lines


def draw_probe_screen(image_module, pin_lines, result_lines, status_text):
    """绘制 I2C 诊断结果画面。

    参数：
    - image_module：MaixPy `image` 模块；
    - pin_lines：pinmap 支持项显示文本；
    - result_lines：scan/write 结果显示文本；
    - status_text：底部状态提示，例如 `tap to rescan`。

    返回值：
    - 可直接传给 `display.show()` 的图像对象。

    副作用：无。函数只创建并绘制一张新图像。
    """
    img = image_module.Image(
        SCREEN_WIDTH,
        SCREEN_HEIGHT,
        image_module.Format.FMT_RGB888,
        image_module.COLOR_BLACK,
    )
    y_pos = 4

    img.draw_string(4, y_pos, "MaixCAM2 I2C PROBE 0x{:02X}".format(TARGET_ADDR), color=image_module.COLOR_YELLOW, scale=TEXT_SCALE)
    y_pos += LINE_HEIGHT + 2

    for line in pin_lines:
        img.draw_string(4, y_pos, clip_text(line, 42), color=image_module.COLOR_WHITE, scale=TEXT_SCALE)
        y_pos += LINE_HEIGHT

    y_pos += 2
    for line in result_lines:
        color = image_module.COLOR_GREEN if "FOUND 42" in line or "write: OK" in line else image_module.COLOR_RED
        if "write: skip" in line:
            color = image_module.COLOR_YELLOW
        img.draw_string(4, y_pos, clip_text(line, 42), color=color, scale=TEXT_SCALE)
        y_pos += LINE_HEIGHT

    img.draw_string(4, SCREEN_HEIGHT - 20, status_text, color=image_module.COLOR_BLUE, scale=TEXT_SCALE)
    return img


def main():
    """MaixCAM2 I2C 诊断程序入口。

    参数：无。
    返回值：无。

    主要流程：
    1. 初始化屏幕与触摸屏；
    2. 上电立即执行一次 I2C 诊断；
    3. 屏幕保持显示结果，点击屏幕后重新诊断一次；
    4. 退出由 MaixPy `app.need_exit()` 控制。

    副作用：
    - 会临时切换 A8/A9/A0/A1 的 pinmap；
    - 扫到 0x42 时会向 ESP32 写入一帧空目标测试帧。
    """
    from maix import app, display, image, time, touchscreen

    disp = display.Display()
    ts = touchscreen.TouchScreen()
    pin_lines, result_lines = run_probe()
    last_touch_ms = 0

    while not app.need_exit():
        now_ms = time.ticks_ms()
        _, _, is_pressed = ts.read()

        if is_pressed and (int(now_ms) - int(last_touch_ms) > RESCAN_DEBOUNCE_MS):
            last_touch_ms = int(now_ms)
            pin_lines, result_lines = run_probe()

        screen = draw_probe_screen(image, pin_lines, result_lines, "tap screen to rescan")
        disp.show(screen)
        time.sleep_ms(100)


if __name__ == "__main__":
    main()
