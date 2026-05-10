# MaixCAM2 颜色追踪基础工程

## 1. 目录用途

这个目录专门存放 MaixCAM2 视觉模块代码，以及后续要联动机械臂抓取时需要参考的机械臂基础代码。

当前基础版本先解决两件事：

1. 在 MaixCAM2 上实现“二值化调参模式”和“追踪抓取模式”切换；
2. 把视觉结果整理成稳定 I2C 二进制协议，便于后续给 ESP32 机械臂状态机消费。

## 2. 当前文件说明

| 路径 | 作用 |
| --- | --- |
| `main.py` | MaixCAM2 主程序，负责摄像头、触摸屏、二值化调参、追踪显示和 I2C 输出 |
| `i2c_probe.py` | MaixCAM2 I2C 诊断程序，用于分别验证 `A8/A9 + I2C7` 和官方示例 `A0/A1 + I2C6` 哪组能扫到 `0x42` |
| `i2c_line_monitor.py` | MaixCAM2 I2C 物理线序连续监视程序，配合 ESP32 `VISION_I2C_LINE_TEST=1` 验证 SCL/SDA 是否真正导通 |
| `vision_core.py` | 纯 Python 核心逻辑，负责 LAB 阈值边界、目标筛选和结果打包 |
| `tests/test_vision_core.py` | 普通 Python 单元测试，不依赖 MaixPy 真机 |
| `reference/` | 机械臂相关参考代码副本，后续改 ESP32 抓取流程时从这里接着做 |

## 3. 当前交互流程

| 模式 | 画面 | 作用 |
| --- | --- | --- |
| `TUNE` | 二值化结果 + 6 个 LAB 字段按钮 + `+/-` 调节按钮 | 用手拿着相机，对准目标颜色，实时调到目标区域稳定发白、背景尽量发黑 |
| `TRACK` | 原图 + 目标框 + 抓取参考十字 + `dx/dy/area` | 输出当前最大目标相对抓取点的偏差，给后续机械臂对准使用 |

当前触摸逻辑：

| 触摸区域 | 效果 |
| --- | --- |
| 底部 `l_min/l_max/a_min/a_max/b_min/b_max` | 选择当前要调的阈值字段 |
| 右侧 `+` / `-` | 按下立即以 `1` 为步进修改一次；继续长按时每 `100ms` 再追加一次 |
| `TRACK` | 保存当前阈值并切到追踪模式 |
| `TUNE` | 从追踪模式切回调参模式 |

## 4. 视觉输出协议

当前采用固定长度 I2C 二进制协议，由 MaixCAM2 作为主机主动写给 ESP32 从机。

字段定义：

| 字段 | 含义 |
| --- | --- |
| `addr` | I2C 从机地址，当前固定 `0x42` |
| `header` | 固定帧头 `0xA5 0x5A` |
| `ver` | 协议版本，当前固定 `0x01` |
| `frame_id` | 16 位递增帧号，用于 ESP32 判断是否拿到新数据 |
| `found` | `1` 表示找到目标，`0` 表示没找到 |
| `dx_px` | 目标中心相对抓取参考点的 X 像素偏差，右正左负 |
| `dy_px` | 目标中心相对抓取参考点的 Y 像素偏差，下正上负 |
| `angle_deg` | 目标角度，当前版本没有可靠角度时可保持 `0` |
| `score` | 0-100 的粗略分数，当前用目标面积映射 |
| `area` | 目标像素面积 |
| `checksum` | 前 15 字节求和后取低 8 位 |

固定布局：

```text
Byte0-1   : A5 5A
Byte2     : 01
Byte3     : found flags
Byte4-5   : frame_id
Byte6-7   : dx_px
Byte8-9   : dy_px
Byte10-11 : angle_deg
Byte12    : score
Byte13-14 : area
Byte15    : checksum
```

这条协议当前只负责“报告看到什么”，不会让 MaixCAM2 直接驱动机械臂动作。机械臂动作所有权仍然保留在 ESP32 侧。

## 5. 颜色阈值方案

当前按 MaixPy 官方 `find_blobs`/`binary` 的 LAB 阈值风格实现：

| 通道 | 范围 |
| --- | --- |
| `L` | `0 ~ 100` |
| `A` | `-128 ~ 127` |
| `B` | `-128 ~ 127` |

默认值改为黄色，当前按你给出的更通用黄色阈值范围作为启动值：

```python
(55, 100, -80, 20, 40, 127)
```

这只是默认起点，不代表你现场最终要追踪的黄色一定就是这组值。实际使用时仍应在 `TUNE` 页面按现场光照重新调。

## 6. 后续机械臂联动建议

当前基础版本先把“找目标”和“调颜色”稳定下来。下一步建议按下面路径推进：

1. MaixCAM2 通过 `A1/A0 -> I2C6` 持续输出固定长度视觉结果帧；
2. ESP32 通过 `GPIO16/GPIO17` 作为 I2C 从机接收视觉结果，维护 `等待目标 -> 对准 -> 下降 -> 吸取 -> 抬起 -> 放置 -> 释放` 状态机；
3. STM32F407 只保留上层控制命令，例如 `ARMGRAB/ARMSTOP/ARMSTATUS`，不要让 F4 直接解释像素。

原因很直接：你的机械臂主控本来就是 ESP32，视觉闭环和吸盘动作也更适合放在 ESP32 同一侧，避免 F4、ESP32、MaixCAM2 三边互相抢状态所有权。

## 7. 官方资料依据

当前实现主要参考了 MaixPy 官方文档中的这些能力：

| 能力 | 官方链接 |
| --- | --- |
| 颜色块查找 `find_blobs` | `https://wiki.sipeed.com/maixpy/doc/en/vision/find_blobs.html` |
| 图像接口 `image`（含 `binary/draw_rect/draw_cross/draw_string`） | `https://wiki.sipeed.com/maixpy/api/maix/image.html` |
| I2C 外设与 MaixCAM2 I2C 配置 | `https://wiki.sipeed.com/maixpy/api/maix/peripheral/i2c.html` |
| 触摸屏接口 `TouchScreen` | `https://wiki.sipeed.com/maixpy/api/maix/touchscreen.html` |

补充一个这次已经踩到的关键点：

| 事项 | 官方结论 |
| --- | --- |
| 屏幕触摸坐标与图像坐标是否总是相同 | 不是。官方文档要求在图像尺寸和屏幕尺寸不一致时，使用 `image.resize_map_pos_reverse` 或同类方法做坐标映射 |
| MaixCAM2 当前支持的屏幕 | 官方显示文档说明，MaixCAM2 目前配套的是 `2.4-inch 640x480` 电容触摸屏 |

## 8. I2C 真机诊断

如果追踪页显示 `SCAN NONE`、`NO 42` 或 `I2C ERR`，先不要继续改抓取逻辑，建议单独运行 `i2c_probe.py`。这个程序不启动摄像头，只验证 MaixCAM2 到 ESP32 控制板的 I2C 电气链路。

| 诊断项 | 含义 |
| --- | --- |
| `I2C6 A1/A0: FOUND 42` | 当前主程序使用的 `A1/A0 + I2C6` 能看到 ESP32 从机地址，MaixCAM2 侧 I2C 配置基本正确 |
| `I2C6 A1/A0: NO 42` 且 `I2C7 A8/A9: FOUND 42` | 当前接线或引脚选择更适合前一版 `A8/A9 + I2C7`，主程序可再切回 I2C7 |
| 两组都是 `NO 42` | 大概率是接线方向、SDA/SCL 反接、未共地、ESP32 没烧当前从机程序、插头接触或供电问题 |
| `ERR set ... failed` | 当前 MaixCAM2 固件的 pinmap 不支持这组复用功能，需要换引脚或升级/确认固件 |

`i2c_probe.py` 扫到 `0x42` 后会写入一帧“未发现目标”的合法空目标测试帧，只用于确认 `writeto()` 链路，不会要求机械臂动作。

## 9. 当前限制

| 项目 | 说明 |
| --- | --- |
| 真机验证 | 当前仓库环境无法直接运行 MaixPy，只做了 Python 逻辑测试和语法检查 |
| 角度 | 当前 `angle_deg` 先保留接口，后续若现场确实需要角度，再基于 blob 旋转信息或拟合结果细化 |
| 机械臂动作 | 当前这一步只完成 MaixCAM2 到 ESP32 的 I2C 对接和结果缓存，还没有启用自动抓取状态机 |
| 目标选择 | 当前只取“面积最大且偏差不过大”的目标，适合单目标手持调试，不适合复杂多目标排序 |
