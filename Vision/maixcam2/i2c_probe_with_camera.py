"""MaixCAM2 启动摄像头后的 I2C6 诊断程序。

用途：
1. `i2c_probe.py` 已经证明不开摄像头时 I2C6 A1/A0 能找到 ESP32 `0x42`；
2. `main.py` 会先启动摄像头再使用 I2C6，若这时扫不到地址，就需要确认
   摄像头驱动是否影响了 I2C6；
3. 本脚本刻意先初始化摄像头并读取几帧，再复用 `i2c_probe.run_probe()` 做同样诊断。

运行方式：
- 把本文件和 `i2c_probe.py` 放在 MaixCAM2 同一目录；
- 运行本文件，观察串口是否仍有 `FOUND 42`。
"""

CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
WARMUP_FRAMES = 5


def main():
    """启动摄像头后执行一次 I2C6 探测。

    参数：无。
    返回值：无。

    主要流程：
    1. 初始化 MaixCAM2 摄像头和屏幕，模拟 `main.py` 的硬件占用状态；
    2. 读取数帧让摄像头完成自动曝光和底层初始化；
    3. 调用 `i2c_probe.run_probe()` 探测 I2C6 A1/A0 是否还能看到 ESP32 `0x42`；
    4. 把结果显示在屏幕并打印到串口。

    副作用：会占用摄像头、屏幕和 I2C6 引脚，仅用于诊断。
    """
    from maix import app, camera, display, image, time, touchscreen

    import i2c_probe

    cam = camera.Camera(CAMERA_WIDTH, CAMERA_HEIGHT)
    disp = display.Display()
    ts = touchscreen.TouchScreen()

    print("[I2C_CAMERA_PROBE] camera warmup begin")
    for _ in range(WARMUP_FRAMES):
        frame = cam.read()
        disp.show(frame, fit=image.Fit.FIT_CONTAIN)
        time.sleep_ms(50)
    print("[I2C_CAMERA_PROBE] camera warmup done")

    pin_lines, result_lines = i2c_probe.run_probe()

    while not app.need_exit():
        _, _, is_pressed = ts.read()
        if is_pressed:
            pin_lines, result_lines = i2c_probe.run_probe()

        screen = i2c_probe.draw_probe_screen(
            image,
            pin_lines,
            result_lines,
            "camera active, tap to rescan",
        )
        disp.show(screen)
        time.sleep_ms(100)


if __name__ == "__main__":
    main()
