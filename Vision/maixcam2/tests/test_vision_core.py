import importlib
import sys
import types
import unittest

from Vision.maixcam2.vision_core import (
    BlobCandidate,
    I2C_FRAME_LENGTH,
    I2C_SLAVE_ADDR,
    LongPressAdjustState,
    ThresholdConfig,
    clamp_threshold_value,
    consume_long_press_adjust_step,
    map_display_point_to_image,
    pack_i2c_result_frame,
    pack_result_line,
    parse_i2c_result_frame,
    pick_largest_valid_blob,
)


class VisionCoreTest(unittest.TestCase):
    def test_clamp_threshold_value_limits_lab_channel_range(self):
        self.assertEqual(clamp_threshold_value(120, 0, 100), 100)
        self.assertEqual(clamp_threshold_value(-200, -128, 127), -128)
        self.assertEqual(clamp_threshold_value(64, -128, 127), 64)

    def test_threshold_config_keeps_lower_bound_not_greater_than_upper_bound(self):
        config = ThresholdConfig(l_min=80, l_max=90, a_min=20, a_max=30, b_min=40, b_max=50)

        config.adjust_channel("l_min", 30)
        self.assertEqual(config.l_min, 90)

        config.adjust_channel("a_max", -40)
        self.assertEqual(config.a_max, 20)

    def test_threshold_config_defaults_to_yellow_tracking(self):
        config = ThresholdConfig()

        self.assertEqual(config.as_tuple(), (55, 100, -80, 20, 40, 127))

    def test_pick_largest_valid_blob_filters_small_or_far_targets(self):
        blobs = [
            BlobCandidate(x=10, y=10, w=4, h=5, pixels=20, cx=12, cy=12, angle_deg=0),
            BlobCandidate(x=20, y=20, w=20, h=20, pixels=300, cx=30, cy=30, angle_deg=10),
            BlobCandidate(x=40, y=40, w=30, h=30, pixels=800, cx=55, cy=55, angle_deg=15),
        ]

        target = pick_largest_valid_blob(blobs, min_pixels=100, pickup_u=50, pickup_v=50, max_offset_px=80)

        self.assertIsNotNone(target)
        self.assertEqual(target.area, 800)
        self.assertEqual(target.dx_px, 5)
        self.assertEqual(target.dy_px, 5)

    def test_pack_result_line_uses_stable_csv_protocol(self):
        line = pack_result_line(
            frame_id=7,
            found=True,
            dx_px=-12,
            dy_px=34,
            angle_deg=5,
            score=88,
            area=456,
        )

        self.assertEqual(line, "V1,7,1,-12,34,5,88,456\n")

    def test_map_display_point_to_image_handles_maixcam2_640x480_screen(self):
        image_x, image_y = map_display_point_to_image(
            image_width=320,
            image_height=240,
            display_width=640,
            display_height=480,
            display_x=500,
            display_y=40,
        )

        self.assertEqual((image_x, image_y), (250, 20))

    def test_consume_long_press_adjust_step_fires_every_100ms(self):
        state = LongPressAdjustState()

        self.assertEqual(consume_long_press_adjust_step(state, "plus", 1000, repeat_ms=100), 1)
        self.assertEqual(consume_long_press_adjust_step(state, "plus", 1050, repeat_ms=100), 0)
        self.assertEqual(consume_long_press_adjust_step(state, "plus", 1099, repeat_ms=100), 0)
        self.assertEqual(consume_long_press_adjust_step(state, "plus", 1100, repeat_ms=100), 1)
        self.assertEqual(consume_long_press_adjust_step(state, "plus", 1199, repeat_ms=100), 0)
        self.assertEqual(consume_long_press_adjust_step(state, "plus", 1200, repeat_ms=100), 1)

    def test_consume_long_press_adjust_step_resets_when_button_changes(self):
        state = LongPressAdjustState()

        self.assertEqual(consume_long_press_adjust_step(state, "minus", 300, repeat_ms=100), 1)
        self.assertEqual(consume_long_press_adjust_step(state, "plus", 400, repeat_ms=100), 1)
        self.assertEqual(consume_long_press_adjust_step(state, "plus", 450, repeat_ms=100), 0)
        self.assertEqual(consume_long_press_adjust_step(state, "plus", 500, repeat_ms=100), 1)

    def test_consume_long_press_adjust_step_stops_when_released(self):
        state = LongPressAdjustState()

        self.assertEqual(consume_long_press_adjust_step(state, "plus", 0), 1)
        self.assertEqual(consume_long_press_adjust_step(state, None, 100), 0)
        self.assertEqual(consume_long_press_adjust_step(state, "plus", 200), 1)

    def test_pack_i2c_result_frame_has_stable_layout_and_checksum(self):
        payload = pack_i2c_result_frame(
            frame_id=0x1234,
            found=True,
            dx_px=-12,
            dy_px=34,
            angle_deg=5,
            score=88,
            area=456,
        )

        self.assertEqual(len(payload), I2C_FRAME_LENGTH)
        self.assertEqual(payload[0], 0xA5)
        self.assertEqual(payload[1], 0x5A)
        self.assertEqual(payload[2], 0x01)
        self.assertEqual(payload[3], 0x01)
        self.assertEqual(payload[4], 0x34)
        self.assertEqual(payload[5], 0x12)
        self.assertEqual(payload[-1], sum(payload[:-1]) & 0xFF)

    def test_parse_i2c_result_frame_round_trip(self):
        payload = pack_i2c_result_frame(
            frame_id=9,
            found=False,
            dx_px=7,
            dy_px=-11,
            angle_deg=-25,
            score=42,
            area=321,
        )

        frame = parse_i2c_result_frame(payload)

        self.assertEqual(frame["slave_addr"], I2C_SLAVE_ADDR)
        self.assertEqual(frame["frame_id"], 9)
        self.assertFalse(frame["found"])
        self.assertEqual(frame["dx_px"], 7)
        self.assertEqual(frame["dy_px"], -11)
        self.assertEqual(frame["angle_deg"], -25)
        self.assertEqual(frame["score"], 42)
        self.assertEqual(frame["area"], 321)


class VisionMainModuleTest(unittest.TestCase):
    """验证 `main.py` 的关键运行时约束。"""

    def setUp(self):
        """为导入 `main.py` 准备最小化的 MaixPy 假模块。"""
        self._saved_main_module = sys.modules.pop("Vision.maixcam2.main", None)
        self._saved_maix_module = sys.modules.get("maix")

        fake_maix = types.ModuleType("maix")
        fake_maix.app = types.SimpleNamespace()
        fake_maix.camera = types.SimpleNamespace()
        fake_maix.display = types.SimpleNamespace()
        fake_maix.time = types.SimpleNamespace()
        fake_maix.touchscreen = types.SimpleNamespace()
        fake_maix.image = types.SimpleNamespace(
            COLOR_BLACK=0,
            COLOR_WHITE=1,
            COLOR_RED=2,
            COLOR_GREEN=3,
            COLOR_BLUE=4,
            COLOR_YELLOW=5,
            Fit=types.SimpleNamespace(FIT_CONTAIN=0),
        )
        sys.modules["maix"] = fake_maix

    def tearDown(self):
        """恢复 `sys.modules`，避免影响其它测试。"""
        sys.modules.pop("Vision.maixcam2.main", None)
        if self._saved_main_module is not None:
            sys.modules["Vision.maixcam2.main"] = self._saved_main_module

        if self._saved_maix_module is None:
            sys.modules.pop("maix", None)
        else:
            sys.modules["maix"] = self._saved_maix_module

    def test_main_module_uses_100ms_long_press_repeat(self):
        main_module = importlib.import_module("Vision.maixcam2.main")

        self.assertEqual(main_module.LONG_PRESS_REPEAT_MS, 100)

    def test_main_module_uses_i2c7_pinmap_defaults(self):
        main_module = importlib.import_module("Vision.maixcam2.main")

        self.assertEqual(main_module.I2C_BUS_ID, 6)
        self.assertEqual(main_module.I2C_SCL_PIN_NAME, "A1")
        self.assertEqual(main_module.I2C_SDA_PIN_NAME, "A0")
        self.assertEqual(main_module.I2C_SCL_FUNC_NAME, "I2C6_SCL")
        self.assertEqual(main_module.I2C_SDA_FUNC_NAME, "I2C6_SDA")

    def test_prepare_track_i2c_state_delays_first_probe(self):
        main_module = importlib.import_module("Vision.maixcam2.main")

        comm_state = main_module.I2CCommState(
            last_write_ok=False,
            last_error_text="old err",
            last_scan_ok=True,
            target_addr_visible=True,
        )

        next_scan_ms, next_write_ms = main_module.prepare_track_i2c_state(comm_state, now_ms=1000)

        self.assertEqual(next_scan_ms, 1000 + main_module.I2C_TRACK_ENTER_DIAG_DELAY_MS)
        self.assertEqual(next_write_ms, 1000 + main_module.I2C_TRACK_ENTER_DIAG_DELAY_MS)
        self.assertIsNone(comm_state.last_write_ok)
        self.assertIsNone(comm_state.last_scan_ok)
        self.assertIsNone(comm_state.target_addr_visible)
        self.assertEqual(comm_state.last_error_text, "")

    def test_set_i2c_worker_track_mode_keeps_link_off_on_track_entry(self):
        main_module = importlib.import_module("Vision.maixcam2.main")

        worker_state = main_module.I2CWorkerState()
        comm_state = main_module.I2CCommState(last_write_ok=False, last_scan_ok=True, target_addr_visible=True)

        main_module.set_i2c_worker_track_mode(worker_state, comm_state, now_ms=500, enabled=True)

        self.assertTrue(worker_state.track_mode_active)
        self.assertFalse(worker_state.link_enabled)
        self.assertFalse(worker_state.force_reprobe)
        self.assertEqual(worker_state.pending_frame, b"")
        self.assertFalse(comm_state.link_enabled)
        self.assertIsNone(comm_state.last_write_ok)
        self.assertIsNone(comm_state.last_scan_ok)

    def test_set_i2c_worker_link_enabled_requests_reprobe(self):
        main_module = importlib.import_module("Vision.maixcam2.main")

        worker_state = main_module.I2CWorkerState(track_mode_active=True)
        comm_state = main_module.I2CCommState()

        main_module.set_i2c_worker_link_enabled(worker_state, comm_state, now_ms=600, enabled=True)

        self.assertTrue(worker_state.link_enabled)
        self.assertTrue(worker_state.force_reprobe)
        self.assertTrue(comm_state.link_enabled)
        self.assertIsNone(comm_state.last_write_ok)
        self.assertIsNone(comm_state.last_scan_ok)

    def test_submit_i2c_result_frame_only_keeps_latest_payload(self):
        main_module = importlib.import_module("Vision.maixcam2.main")

        worker_state = main_module.I2CWorkerState(track_mode_active=True)

        main_module.submit_i2c_result_frame(worker_state, b"\x01\x02")
        main_module.submit_i2c_result_frame(worker_state, b"\x03\x04")

        self.assertEqual(worker_state.pending_frame, b"\x03\x04")

    def test_can_write_i2c_result_requires_scan_success_and_addr_visible(self):
        main_module = importlib.import_module("Vision.maixcam2.main")

        self.assertFalse(main_module.can_write_i2c_result(main_module.I2CCommState()))
        self.assertFalse(
            main_module.can_write_i2c_result(
                main_module.I2CCommState(last_scan_ok=True, target_addr_visible=False)
            )
        )
        self.assertFalse(
            main_module.can_write_i2c_result(
                main_module.I2CCommState(last_scan_ok=False, target_addr_visible=True)
            )
        )
        self.assertTrue(
            main_module.can_write_i2c_result(
                main_module.I2CCommState(last_scan_ok=True, target_addr_visible=True)
            )
        )

    def test_write_i2c_frame_marks_comm_ok_after_successful_write(self):
        main_module = importlib.import_module("Vision.maixcam2.main")

        class DummyBus:
            """模拟写入成功的 I2C 总线。"""

            def writeto(self, slave_addr, frame_bytes):
                return len(frame_bytes)

        comm_state = main_module.I2CCommState(link_enabled=True)
        payload = b"\x01\x02\x03"

        write_ok = main_module.write_i2c_frame(DummyBus(), payload, comm_state, now_ms=1234)
        status_text, status_color = main_module.get_i2c_status_display(comm_state, now_ms=1234)

        self.assertTrue(write_ok)
        self.assertEqual(status_text, "I2C OK")
        self.assertEqual(status_color, main_module.COLOR_GREEN)

    def test_scan_i2c_bus_marks_target_visible_when_addr_42_present(self):
        main_module = importlib.import_module("Vision.maixcam2.main")

        class DummyBus:
            """模拟能在总线上扫描到 0x42 的 I2C 总线。"""

            def scan(self):
                return [0x42, 0x68]

        comm_state = main_module.I2CCommState(link_enabled=True)

        scan_ok = main_module.scan_i2c_bus(DummyBus(), comm_state, now_ms=100)
        scan_text, scan_color = main_module.get_i2c_scan_display(comm_state)

        self.assertTrue(scan_ok)
        self.assertTrue(comm_state.target_addr_visible)
        self.assertEqual(scan_text, "SCAN 42 OK")
        self.assertEqual(scan_color, main_module.COLOR_GREEN)

    def test_scan_i2c_bus_prefers_targeted_scan_when_supported(self):
        main_module = importlib.import_module("Vision.maixcam2.main")

        class DummyBus:
            """模拟支持 `scan(addr)` 的 MaixPy I2C 总线。"""

            def __init__(self):
                """记录最近一次扫描地址，便于断言主程序优先定向扫描 0x42。"""
                self.last_scan_addr = None

            def scan(self, slave_addr=-1):
                """返回调用方指定的地址，模拟定向扫描命中目标从机。"""
                self.last_scan_addr = slave_addr
                return [slave_addr]

        dummy_bus = DummyBus()
        comm_state = main_module.I2CCommState(link_enabled=True)

        scan_ok = main_module.scan_i2c_bus(dummy_bus, comm_state, now_ms=100)

        self.assertTrue(scan_ok)
        self.assertEqual(dummy_bus.last_scan_addr, main_module.I2C_SLAVE_ADDR)
        self.assertTrue(comm_state.target_addr_visible)

    def test_scan_i2c_bus_marks_bus_none_when_no_device_found(self):
        main_module = importlib.import_module("Vision.maixcam2.main")

        class DummyBus:
            """模拟空总线扫描结果。"""

            def scan(self):
                return []

        comm_state = main_module.I2CCommState(link_enabled=True)

        scan_ok = main_module.scan_i2c_bus(DummyBus(), comm_state, now_ms=100)
        scan_text, scan_color = main_module.get_i2c_scan_display(comm_state)
        detail_text, detail_color = main_module.get_i2c_detail_display(comm_state)

        self.assertTrue(scan_ok)
        self.assertFalse(comm_state.target_addr_visible)
        self.assertEqual(scan_text, "SCAN NONE")
        self.assertEqual(scan_color, main_module.COLOR_RED)
        self.assertEqual(detail_text, "")
        self.assertEqual(detail_color, main_module.COLOR_WHITE)

    def test_write_i2c_frame_marks_comm_error_after_failed_write(self):
        main_module = importlib.import_module("Vision.maixcam2.main")

        class DummyBus:
            """模拟写入失败的 I2C 总线。"""

            def writeto(self, slave_addr, frame_bytes):
                raise RuntimeError("nack")

        comm_state = main_module.I2CCommState(link_enabled=True)
        payload = b"\x01\x02\x03"

        write_ok = main_module.write_i2c_frame(DummyBus(), payload, comm_state, now_ms=5678)
        status_text, status_color = main_module.get_i2c_status_display(comm_state, now_ms=5678)

        self.assertFalse(write_ok)
        self.assertEqual(status_text, "I2C ERR")
        self.assertEqual(status_color, main_module.COLOR_RED)

    def test_write_i2c_frame_preserves_negative_return_code_for_debug(self):
        main_module = importlib.import_module("Vision.maixcam2.main")

        class DummyBus:
            """模拟 `writeto()` 返回负错误码的 I2C 总线。"""

            def writeto(self, slave_addr, frame_bytes):
                return -5

        comm_state = main_module.I2CCommState(link_enabled=True)

        write_ok = main_module.write_i2c_frame(DummyBus(), b"\x01\x02", comm_state, now_ms=200)
        detail_text, detail_color = main_module.get_i2c_detail_display(comm_state)

        self.assertFalse(write_ok)
        self.assertEqual(comm_state.last_write_length, -5)
        self.assertEqual(detail_text, "RET -5")
        self.assertEqual(detail_color, main_module.COLOR_RED)

    def test_blob_to_candidate_returns_core_blob_candidate(self):
        main_module = importlib.import_module("Vision.maixcam2.main")

        class DummyBlob:
            """模拟 MaixPy `find_blobs` 返回对象。"""

            def x(self):
                return 11

            def y(self):
                return 22

            def w(self):
                return 33

            def h(self):
                return 44

            def pixels(self):
                return 555

            def cx(self):
                return 66

            def cy(self):
                return 77

            def rotation_deg(self):
                return 88

        candidate = main_module.blob_to_candidate(DummyBlob())

        self.assertIsInstance(candidate, BlobCandidate)
        self.assertEqual(candidate.cx, 66)
        self.assertEqual(candidate.angle_deg, 88)

    def test_smooth_target_for_control_follows_large_motion_quickly(self):
        main_module = importlib.import_module("Vision.maixcam2.main")
        smooth_state = main_module.TargetSmoothState()
        pickup_u = 160
        pickup_v = 120

        left_target = pick_largest_valid_blob(
            [BlobCandidate(x=30, y=100, w=20, h=20, pixels=1000, cx=40, cy=110, angle_deg=0)],
            min_pixels=100,
            pickup_u=pickup_u,
            pickup_v=pickup_v,
            max_offset_px=220,
        )
        right_target = pick_largest_valid_blob(
            [BlobCandidate(x=270, y=100, w=20, h=20, pixels=1100, cx=280, cy=110, angle_deg=0)],
            min_pixels=100,
            pickup_u=pickup_u,
            pickup_v=pickup_v,
            max_offset_px=220,
        )

        first_smoothed = main_module.smooth_target_for_control(left_target, smooth_state, pickup_u, pickup_v)
        second_smoothed = main_module.smooth_target_for_control(right_target, smooth_state, pickup_u, pickup_v)

        self.assertEqual(first_smoothed.dx_px, -120)
        self.assertGreater(second_smoothed.dx_px, 60)
        self.assertLess(second_smoothed.dx_px, 120)


if __name__ == "__main__":
    unittest.main()
