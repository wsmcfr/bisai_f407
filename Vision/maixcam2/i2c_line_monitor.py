"""MaixCAM2 I2C 物理线序连续监视程序。

运行方式：
1. ESP32 端把 `VISION_I2C_LINE_TEST` 改为 1 并重新烧录；
2. MaixCAM2 运行本文件；
3. 观察屏幕和串口中的 I2C6/I2C7 SCL/SDA 电平变化。

这个程序不创建 I2C 总线、不扫描地址，只把候选引脚当 GPIO 输入连续读取。
它用于确认 MaixCAM2 的 A1/A0 或 A8/A9 是否真的连接到了 ESP32 的 GPIO16/GPIO17。
"""

TARGET_REFRESH_MS = 200
SCREEN_WIDTH = 320
SCREEN_HEIGHT = 240
TEXT_SCALE = 0.85


LINE_CANDIDATES = (
    {
        "name": "I2C6 A1/A0",
        "scl_pin": "A1",
        "sda_pin": "A0",
    },
    {
        "name": "I2C7 A8/A9",
        "scl_pin": "A8",
        "sda_pin": "A9",
    },
)


def draw_text(img, x_pos, y_pos, text, color, scale=TEXT_SCALE):
    """在图像上绘制一行诊断文本。

    参数：
    - img：MaixPy 图像对象；
    - x_pos/y_pos：文本左上角坐标；
    - text：要显示的文本；
    - color：文本颜色；
    - scale：字体缩放。

    返回值：无。
    副作用：会修改传入图像，用于实时显示线序状态。
    """
    img.draw_string(x_pos, y_pos, str(text), color=color, scale=scale)


def setup_gpio_input(pinmap_module, gpio_module, pin_name):
    """把 MaixCAM2 一个引脚切到 GPIO 输入模式。

    参数：
    - pinmap_module：MaixPy `pinmap` 模块；
    - gpio_module：MaixPy `gpio` 模块；
    - pin_name：例如 `A1`、`A0`、`A8`、`A9`。

    返回值：
    - 成功时返回 GPIO 对象；
    - 失败时返回错误文本。

    副作用：
    - 会把该引脚从当前功能临时切换为 GPIO 输入；
    - 本脚本只做线序监视，不会再切回 I2C。
    """
    gpio_func = "GPIO{}".format(pin_name)

    try:
        pinmap_module.set_pin_function(pin_name, gpio_func)
        return gpio_module.GPIO(gpio_func, gpio_module.Mode.IN, gpio_module.Pull.PULL_NONE)
    except Exception as exc:
        return "ERR {}".format(exc)


def build_line_inputs(pinmap_module, gpio_module):
    """初始化所有候选 I2C 线的 GPIO 输入对象。

    参数：
    - pinmap_module：MaixPy `pinmap` 模块；
    - gpio_module：MaixPy `gpio` 模块。

    返回值：
    - 列表，每项包含候选名、SCL GPIO 对象或错误、SDA GPIO 对象或错误。

    副作用：
    - 会切换 A1/A0/A8/A9 的 pinmap 到 GPIO 输入，便于连续读电平。
    """
    inputs = []
    for candidate in LINE_CANDIDATES:
        scl_obj = setup_gpio_input(pinmap_module, gpio_module, candidate["scl_pin"])
        sda_obj = setup_gpio_input(pinmap_module, gpio_module, candidate["sda_pin"])
        inputs.append(
            {
                "name": candidate["name"],
                "scl_pin": candidate["scl_pin"],
                "sda_pin": candidate["sda_pin"],
                "scl_obj": scl_obj,
                "sda_obj": sda_obj,
            }
        )
    return inputs


def read_level(gpio_or_error):
    """读取一个 GPIO 输入对象的当前电平。

    参数：
    - gpio_or_error：GPIO 对象，或初始化失败时的错误文本。

    返回值：
    - 成功时返回 0 或 1；
    - 失败时返回错误文本。

    副作用：无。该函数只读取当前电平。
    """
    if isinstance(gpio_or_error, str):
        return gpio_or_error

    try:
        return int(gpio_or_error.value())
    except Exception as exc:
        return "ERR {}".format(exc)


def classify_levels(scl_level, sda_level):
    """根据 SCL/SDA 电平给出线序测试提示。

    参数：
    - scl_level：当前 SCL 读数；
    - sda_level：当前 SDA 读数。

    返回值：
    - 用于屏幕显示的状态文本。

    判定依据：
    - ESP32 线序测试释放总线时应为 `1/1`；
    - ESP32 打印 `pull SCL(GPIO16) low` 时应为 `0/1`；
    - ESP32 打印 `pull SDA(GPIO17) low` 时应为 `1/0`。
    """
    if (scl_level == 1) and (sda_level == 1):
        return "release or not connected"
    if (scl_level == 0) and (sda_level == 1):
        return "SCL low"
    if (scl_level == 1) and (sda_level == 0):
        return "SDA low"
    if (scl_level == 0) and (sda_level == 0):
        return "both low"
    return "read error"


def draw_monitor_screen(image_module, inputs):
    """绘制线序监视画面。

    参数：
    - image_module：MaixPy `image` 模块；
    - inputs：`build_line_inputs()` 返回的 GPIO 输入列表。

    返回值：
    - 可直接传给 `display.show()` 的图像对象。

    副作用：
    - 会读取 GPIO 当前电平；
    - 会把每组读数打印到串口，便于复制日志回来分析。
    """
    img = image_module.Image(
        SCREEN_WIDTH,
        SCREEN_HEIGHT,
        image_module.Format.FMT_RGB888,
        image_module.COLOR_BLACK,
    )

    draw_text(img, 4, 4, "I2C LINE MONITOR", image_module.COLOR_YELLOW)
    draw_text(img, 4, 24, "ESP32 LINE_TEST=1", image_module.COLOR_WHITE, 0.75)
    y_pos = 54

    for item in inputs:
        scl_level = read_level(item["scl_obj"])
        sda_level = read_level(item["sda_obj"])
        state_text = classify_levels(scl_level, sda_level)
        line_text = "{} SCL={} SDA={}".format(item["name"], scl_level, sda_level)

        color = image_module.COLOR_GREEN if state_text in ("SCL low", "SDA low") else image_module.COLOR_WHITE
        draw_text(img, 4, y_pos, line_text, color)
        draw_text(img, 18, y_pos + 18, state_text, image_module.COLOR_BLUE, 0.75)
        print("[LINE_MON] {} scl={} sda={} state={}".format(item["name"], scl_level, sda_level, state_text))
        y_pos += 48

    draw_text(img, 4, SCREEN_HEIGHT - 28, "match ESP32 serial: SCL low / SDA low", image_module.COLOR_YELLOW, 0.65)
    return img


def main():
    """MaixCAM2 I2C 线序监视程序入口。

    参数：无。
    返回值：无。

    主要流程：
    1. 初始化屏幕；
    2. 把 A1/A0/A8/A9 切为 GPIO 输入；
    3. 每 200ms 读取一次电平并显示；
    4. 退出由 MaixPy `app.need_exit()` 控制。

    副作用：
    - 会占用候选 I2C 引脚为 GPIO 输入；
    - 本脚本用于排线，不用于正常视觉追踪。
    """
    from maix import app, display, gpio, image, pinmap, time

    disp = display.Display()
    inputs = build_line_inputs(pinmap, gpio)

    while not app.need_exit():
        disp.show(draw_monitor_screen(image, inputs))
        time.sleep_ms(TARGET_REFRESH_MS)


if __name__ == "__main__":
    main()
