#include "binary_protocol_service.h" /* 引入 MP157-F4 二进制协议定义和解码函数，主机测试直接验证这些公开接口。 */

#include <stdint.h>                   /* 提供 uint8_t、uint16_t 等固定宽度整数类型，保证帧字段宽度稳定。 */
#include <stdio.h>                    /* 提供 printf()，用于在 Windows 主机侧输出失败项。 */
#include <string.h>                   /* 提供 memset() 和 memcpy()，用于构造测试帧和清空负载缓存。 */

static int g_failed_count = 0;        /* 记录当前主机回归测试失败数量，main() 根据该值返回 0 或 1。 */

/**
 * @brief 校验一个 16 位无符号字段是否等于预期值。
 * @param name 当前断言名称，失败时打印到控制台，便于快速定位字段。
 * @param actual 被测函数实际输出值。
 * @param expected 当前协议契约要求的预期值。
 *
 * 返回值：
 *   无返回值；失败时递增 g_failed_count。
 */
static void expect_u16(const char *name, uint16_t actual, uint16_t expected)
{
    if (actual != expected)
    {
        printf("FAIL %s: actual=0x%04X expected=0x%04X\n", name, actual, expected);
        ++g_failed_count;
    }
}

/**
 * @brief 校验一个 int 字段是否等于预期值。
 * @param name 当前断言名称，失败时打印到控制台。
 * @param actual 被测函数实际输出值。
 * @param expected 当前协议契约要求的预期值。
 *
 * 返回值：
 *   无返回值；失败时递增 g_failed_count。
 */
static void expect_int(const char *name, int actual, int expected)
{
    if (actual != expected)
    {
        printf("FAIL %s: actual=%d expected=%d\n", name, actual, expected);
        ++g_failed_count;
    }
}

/**
 * @brief 按协议小端序写入 16 位无符号数。
 * @param buffer 目标缓存，调用方保证至少有 2 字节空间。
 * @param value 需要写入的数值。
 *
 * 返回值：
 *   无返回值；buffer[0] 保存低字节，buffer[1] 保存高字节。
 */
static void write_u16_le(uint8_t *buffer, uint16_t value)
{
    buffer[0] = (uint8_t)(value & 0xFFU);
    buffer[1] = (uint8_t)((value >> 8) & 0xFFU);
}

/**
 * @brief 按协议小端序写入 16 位有符号数。
 * @param buffer 目标缓存，调用方保证至少有 2 字节空间。
 * @param value 需要写入的有符号数值。
 *
 * 返回值：
 *   无返回值；底层复用 write_u16_le() 保证与协议小端格式一致。
 */
static void write_i16_le(uint8_t *buffer, int16_t value)
{
    write_u16_le(buffer, (uint16_t)value);
}

/**
 * @brief 构造一帧带正确 CRC16-CCITT-FALSE 的二进制协议帧。
 * @param frame 输出帧缓存，调用方保证空间足够容纳 10 字节固定开销和 payload。
 * @param command 命令字，例如 START_CYCLE、WEIGHT_CALIBRATE。
 * @param sequence 帧序号，用于测试解析后的 sequence 字段。
 * @param payload 负载指针；payload_length 为 0 时允许为空。
 * @param payload_length 负载长度，单位字节。
 * @return uint16_t 返回完整帧长度。
 *
 * 主要流程：
 *   1. 写入帧头、版本、命令、长度和序号；
 *   2. 拷贝 payload；
 *   3. 对 VERSION 到 PAYLOAD 末尾计算 CRC；
 *   4. 写入小端 CRC 和帧尾。
 */
static uint16_t build_crc_frame(uint8_t *frame,
                                uint8_t command,
                                uint16_t sequence,
                                const uint8_t *payload,
                                uint8_t payload_length)
{
    uint16_t crc;
    uint16_t frame_length;

    frame[0] = BINARY_PROTOCOL_SOF0;
    frame[1] = BINARY_PROTOCOL_SOF1;
    frame[2] = BINARY_PROTOCOL_VERSION;
    frame[3] = command;
    frame[4] = payload_length;
    frame[5] = (uint8_t)(sequence & 0xFFU);
    frame[6] = (uint8_t)((sequence >> 8) & 0xFFU);
    if ((payload != NULL) && (payload_length > 0U))
    {
        (void)memcpy(&frame[7], payload, payload_length);
    }
    crc = BinaryProtocolService_Crc16CcittFalse(&frame[2], (uint16_t)(5U + payload_length));
    frame[7U + payload_length] = (uint8_t)(crc & 0xFFU);
    frame[8U + payload_length] = (uint8_t)((crc >> 8) & 0xFFU);
    frame[9U + payload_length] = BINARY_PROTOCOL_EOF;
    frame_length = (uint16_t)(10U + payload_length);
    return frame_length;
}

/**
 * @brief 验证 CRC16-CCITT-FALSE 标准测试向量。
 *
 * 返回值：
 *   无返回值；失败时记录断言失败。
 */
static void test_crc_standard_vector(void)
{
    static const uint8_t vector[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    uint16_t crc = BinaryProtocolService_Crc16CcittFalse(vector, (uint16_t)sizeof(vector));
    expect_u16("crc16_ccitt_false_123456789", crc, 0x29B1U);
}

/**
 * @brief 验证 START_CYCLE 完整帧能被解析出命令字、序号和负载长度。
 *
 * 返回值：
 *   无返回值；失败时记录断言失败。
 */
static void test_parse_start_cycle_frame(void)
{
    uint8_t payload[6];
    uint8_t frame_buffer[64];
    uint16_t frame_length;
    BinaryProtocol_Frame_t frame;
    BinaryProtocol_ParseStatus_t status;

    write_u16_le(&payload[0], 0x1234U);
    payload[2] = 0x00U;
    write_u16_le(&payload[3], 0x0007U);
    payload[5] = 0x00U;

    frame_length = build_crc_frame(frame_buffer, BINARY_PROTOCOL_CMD_START_CYCLE, 0x0034U, payload, (uint8_t)sizeof(payload));
    status = BinaryProtocolService_ParseFrame(frame_buffer, frame_length, &frame);

    expect_int("parse_start_cycle_status", (int)status, (int)BINARY_PROTOCOL_PARSE_OK);
    expect_int("parse_start_cycle_command", (int)frame.command, (int)BINARY_PROTOCOL_CMD_START_CYCLE);
    expect_u16("parse_start_cycle_sequence", frame.sequence, 0x0034U);
    expect_int("parse_start_cycle_length", (int)frame.payload_length, 6);
}

/**
 * @brief 验证 CRC 被破坏后解析函数返回 CRC 错误。
 *
 * 返回值：
 *   无返回值；失败时记录断言失败。
 */
static void test_parse_crc_error(void)
{
    uint8_t frame_buffer[64];
    uint16_t frame_length;
    BinaryProtocol_Frame_t frame;
    BinaryProtocol_ParseStatus_t status;

    frame_length = build_crc_frame(frame_buffer, BINARY_PROTOCOL_CMD_HELLO, 0x0001U, NULL, 0U);
    frame_buffer[7] ^= 0x55U;
    status = BinaryProtocolService_ParseFrame(frame_buffer, frame_length, &frame);

    expect_int("parse_crc_error_status", (int)status, (int)BINARY_PROTOCOL_PARSE_CRC_ERROR);
}

/**
 * @brief 验证 PAUSE_CYCLE 和 RESUME_CYCLE 负载小端解码。
 *
 * 返回值：
 *   无返回值；失败时记录断言失败。
 */
static void test_decode_pause_resume_payload(void)
{
    uint8_t pause_payload[BINARY_PROTOCOL_PAUSE_CYCLE_PAYLOAD_LENGTH];
    uint8_t resume_payload[BINARY_PROTOCOL_RESUME_CYCLE_PAYLOAD_LENGTH];
    BinaryProtocol_PauseCyclePayload_t pause_cycle;
    BinaryProtocol_ResumeCyclePayload_t resume_cycle;
    uint8_t ok;

    write_u16_le(&pause_payload[0], 0x0044U);
    pause_payload[2] = 0x00U;
    pause_payload[3] = 0x01U;
    ok = BinaryProtocolService_DecodePauseCycle(pause_payload, (uint8_t)sizeof(pause_payload), &pause_cycle);
    expect_int("decode_pause_ok", (int)ok, 1);
    expect_u16("decode_pause_cycle", pause_cycle.cycle_id, 0x0044U);
    expect_int("decode_pause_mode", (int)pause_cycle.pause_mode, 1);

    write_u16_le(&resume_payload[0], 0x0044U);
    resume_payload[2] = 0x01U;
    ok = BinaryProtocolService_DecodeResumeCycle(resume_payload, (uint8_t)sizeof(resume_payload), &resume_cycle);
    expect_int("decode_resume_ok", (int)ok, 1);
    expect_u16("decode_resume_cycle", resume_cycle.cycle_id, 0x0044U);
    expect_int("decode_resume_mode", (int)resume_cycle.resume_mode, 1);
}

/**
 * @brief 验证 VISION_POS 负载的坐标和置信度解码。
 *
 * 返回值：
 *   无返回值；失败时记录断言失败。
 */
static void test_decode_vision_pos_payload(void)
{
    uint8_t payload[BINARY_PROTOCOL_VISION_POS_PAYLOAD_LENGTH];
    BinaryProtocol_VisionPosPayload_t vision_pos;
    uint8_t ok;

    (void)memset(payload, 0, sizeof(payload));
    write_u16_le(&payload[0], 0x0022U);
    write_u16_le(&payload[2], 0x0102U);
    payload[4] = 0x03U;
    payload[5] = 0x01U;
    write_i16_le(&payload[6], 205);
    write_i16_le(&payload[8], 240);
    write_i16_le(&payload[10], 312);
    write_i16_le(&payload[12], 205);
    write_i16_le(&payload[14], 260);
    write_i16_le(&payload[16], 150);
    write_i16_le(&payload[18], 104);
    write_i16_le(&payload[20], 108);
    payload[22] = 91U;
    payload[23] = 0U;
    payload[24] = 0x78U;
    payload[25] = 0x56U;
    payload[26] = 0x34U;
    payload[27] = 0x12U;

    ok = BinaryProtocolService_DecodeVisionPos(payload, (uint8_t)sizeof(payload), &vision_pos);

    expect_int("decode_vision_pos_ok", (int)ok, 1);
    expect_u16("decode_vision_pos_cycle", vision_pos.cycle_id, 0x0022U);
    expect_int("decode_vision_pos_axis", (int)vision_pos.axis_px, 205);
    expect_int("decode_vision_pos_target", (int)vision_pos.target_px, 240);
    expect_int("decode_vision_pos_confidence", (int)vision_pos.confidence, 91);
}

/**
 * @brief 验证 STEPPER_PARAM_SET 三台电机运行参数解码。
 *
 * 返回值：
 *   无返回值；失败时记录断言失败。
 */
static void test_decode_stepper_param_payload(void)
{
    uint8_t payload[BINARY_PROTOCOL_STEPPER_PARAM_PAYLOAD_LENGTH];
    BinaryProtocol_StepperParamPayload_t stepper_param;
    uint8_t ok;

    (void)memset(payload, 0, sizeof(payload));
    write_u16_le(&payload[0], 0x0000U);
    payload[2] = 3U;
    payload[3] = 0U;

    payload[4] = 1U;
    payload[5] = 1U;
    write_u16_le(&payload[6], 20U);
    write_u16_le(&payload[8], 300U);
    write_u16_le(&payload[10], 40U);
    payload[12] = 1U;

    payload[13] = 2U;
    payload[14] = 3U;
    write_u16_le(&payload[15], 5U);
    write_u16_le(&payload[17], 137U);
    write_u16_le(&payload[19], 0U);
    payload[21] = 1U;

    payload[22] = 3U;
    payload[23] = 2U;
    write_u16_le(&payload[24], 5U);
    write_u16_le(&payload[26], 5000U);
    write_u16_le(&payload[28], 0U);
    payload[30] = 0xFFU;

    ok = BinaryProtocolService_DecodeStepperParam(payload, (uint8_t)sizeof(payload), &stepper_param);

    expect_int("decode_stepper_param_ok", (int)ok, 1);
    expect_u16("decode_stepper_param_cycle", stepper_param.cycle_id, 0x0000U);
    expect_int("decode_stepper_param_count", (int)stepper_param.motor_count, 3);
    expect_int("decode_stepper_param_role0", (int)stepper_param.motors[0].role_id, 1);
    expect_u16("decode_stepper_param_scan0", stepper_param.motors[0].scan_speed_rpm, 40U);
    expect_int("decode_stepper_param_lateral_addr", (int)stepper_param.motors[1].address, 3);
    expect_u16("decode_stepper_param_speed1", stepper_param.motors[1].normal_speed_rpm, 137U);
    expect_u16("decode_stepper_param_scan1", stepper_param.motors[1].scan_speed_rpm, 0U);
    expect_int("decode_stepper_param_z_addr", (int)stepper_param.motors[2].address, 2);
    expect_u16("decode_stepper_param_speed2", stepper_param.motors[2].normal_speed_rpm, 5000U);
    expect_u16("decode_stepper_param_scan2", stepper_param.motors[2].scan_speed_rpm, 0U);
    expect_int("decode_stepper_param_direction2", (int)stepper_param.motors[2].direction, -1);
}

/**
 * @brief 验证 WEIGHT_CALIBRATE 5 字节负载能正确解出 cycle、克重和 flags。
 *
 * 返回值：
 *   无返回值；失败时记录断言失败。
 */
static void test_decode_weight_calibration_payload(void)
{
    uint8_t payload[BINARY_PROTOCOL_WEIGHT_CALIBRATION_PAYLOAD_LENGTH];
    BinaryProtocol_WeightCalibrationPayload_t weight_calibration;
    uint8_t ok;

    (void)memset(payload, 0, sizeof(payload));
    write_u16_le(&payload[0], 0x0000U);
    write_u16_le(&payload[2], 1000U);
    payload[4] = 0U;

    ok = BinaryProtocolService_DecodeWeightCalibration(payload,
                                                       (uint8_t)sizeof(payload),
                                                       &weight_calibration);

    expect_int("decode_weight_calibration_ok", (int)ok, 1);
    expect_u16("decode_weight_calibration_cycle", weight_calibration.cycle_id, 0x0000U);
    expect_u16("decode_weight_calibration_known", weight_calibration.known_weight_g, 1000U);
    expect_int("decode_weight_calibration_flags", (int)weight_calibration.flags, 0);
}

/**
 * @brief 验证 WEIGHT_CALIBRATE 长度错误时必须拒绝解码。
 *
 * 返回值：
 *   无返回值；失败时记录断言失败。
 */
static void test_decode_weight_calibration_rejects_wrong_length(void)
{
    uint8_t payload[BINARY_PROTOCOL_WEIGHT_CALIBRATION_PAYLOAD_LENGTH];
    BinaryProtocol_WeightCalibrationPayload_t weight_calibration;
    uint8_t ok;

    (void)memset(payload, 0, sizeof(payload));
    ok = BinaryProtocolService_DecodeWeightCalibration(payload,
                                                       (uint8_t)(sizeof(payload) - 1U),
                                                       &weight_calibration);

    expect_int("decode_weight_calibration_bad_len", (int)ok, 0);
}

/**
 * @brief 验证 MODEL_READY 使用独立命令号和固定 12 字节负载。
 *
 * 返回值：
 *   无返回值；失败时记录断言失败。
 */
static void test_decode_model_ready_payload(void)
{
    uint8_t payload[BINARY_PROTOCOL_MODEL_READY_PAYLOAD_LENGTH];
    BinaryProtocol_ModelReadyPayload_t model_ready;
    uint8_t ok;

    (void)memset(payload, 0, sizeof(payload));
    write_u16_le(&payload[0], 0x0055U);
    payload[2] = 1U;
    payload[3] = 2U;
    payload[4] = 3U;
    payload[5] = 87U;
    write_u16_le(&payload[6], 12U);
    write_u16_le(&payload[8], 346U);
    write_u16_le(&payload[10], 0x0007U);

    ok = BinaryProtocolService_DecodeModelReady(payload,
                                                (uint8_t)sizeof(payload),
                                                &model_ready);

    expect_int("model_ready_command_is_0x32",
               (int)BINARY_PROTOCOL_CMD_MODEL_READY,
               0x32);
    expect_int("model_ready_not_weight_calibrate",
               (int)(BINARY_PROTOCOL_CMD_MODEL_READY == BINARY_PROTOCOL_CMD_WEIGHT_CALIBRATE),
               0);
    expect_int("decode_model_ready_ok", (int)ok, 1);
    expect_u16("decode_model_ready_cycle", model_ready.cycle_id, 0x0055U);
    expect_int("decode_model_ready_result", (int)model_ready.model_result, 1);
    expect_int("decode_model_ready_part_type", (int)model_ready.part_type, 2);
    expect_int("decode_model_ready_defect_type", (int)model_ready.defect_type, 3);
    expect_int("decode_model_ready_confidence", (int)model_ready.top1_confidence, 87);
    expect_u16("decode_model_ready_image_seq", model_ready.image_seq, 12U);
    expect_u16("decode_model_ready_model_ms", model_ready.model_ms, 346U);
    expect_u16("decode_model_ready_options", model_ready.option_bits, 0x0007U);
}

/**
 * @brief 验证 ARM_JOB_START 负载能正确解出任务和最终分拣提示。
 *
 * 返回值：
 *   无返回值；失败时记录断言失败。
 */
static void test_decode_arm_job_start_payload(void)
{
    uint8_t payload[BINARY_PROTOCOL_ARM_JOB_START_PAYLOAD_LENGTH];
    BinaryProtocol_ArmJobStartPayload_t arm_job;
    uint8_t ok;

    (void)memset(payload, 0, sizeof(payload));
    write_u16_le(&payload[0], 0x0055U);
    write_u16_le(&payload[2], 0x0102U);
    payload[4] = 0U;
    payload[5] = 2U;
    payload[6] = 3U;
    write_u16_le(&payload[7], 0x0007U);

    ok = BinaryProtocolService_DecodeArmJobStart(payload,
                                                 (uint8_t)sizeof(payload),
                                                 &arm_job);

    expect_int("decode_arm_job_ok", (int)ok, 1);
    expect_u16("decode_arm_job_cycle", arm_job.cycle_id, 0x0055U);
    expect_u16("decode_arm_job_id", arm_job.job_id, 0x0102U);
    expect_int("decode_arm_job_profile", (int)arm_job.job_profile, 0);
    expect_int("decode_arm_job_part_type", (int)arm_job.part_type, 2);
    expect_int("decode_arm_job_final_bin", (int)arm_job.final_bin_hint, 3);
    expect_u16("decode_arm_job_options", arm_job.option_bits, 0x0007U);
}

/**
 * @brief 验证 WEIGHT_RESULT 上报负载长度和关键字段布局稳定。
 *
 * 返回值：
 *   无返回值；失败时记录断言失败。
 */
static void test_decode_weight_result_payload(void)
{
    uint8_t payload[BINARY_PROTOCOL_WEIGHT_RESULT_PAYLOAD_LENGTH];
    BinaryProtocol_WeightResultPayload_t weight_result;
    uint8_t ok;

    (void)memset(payload, 0, sizeof(payload));
    write_u16_le(&payload[0], 0x0055U);
    write_u16_le(&payload[2], 0x0009U);
    payload[4] = 1U;
    payload[5] = 1U;
    payload[6] = 0x78U;
    payload[7] = 0x56U;
    payload[8] = 0x34U;
    payload[9] = 0x12U;
    payload[10] = 0x10U;
    payload[11] = 0x27U;
    payload[12] = 0x00U;
    payload[13] = 0x00U;
    payload[14] = 0x33U;
    payload[15] = 0x22U;
    payload[16] = 0x11U;
    payload[17] = 0x00U;
    write_u16_le(&payload[18], 10U);
    write_u16_le(&payload[20], 200U);
    write_u16_le(&payload[22], 1500U);
    payload[24] = 0x44U;
    payload[25] = 0x33U;
    payload[26] = 0x22U;
    payload[27] = 0x11U;

    ok = BinaryProtocolService_DecodeWeightResult(payload,
                                                  (uint8_t)sizeof(payload),
                                                  &weight_result);

    expect_int("decode_weight_result_ok", (int)ok, 1);
    expect_u16("decode_weight_result_cycle", weight_result.cycle_id, 0x0055U);
    expect_u16("decode_weight_result_sample", weight_result.sample_id, 9U);
    expect_int("decode_weight_result_stable", (int)weight_result.stable, 1);
    expect_int("decode_weight_result_decision", (int)weight_result.decision, 1);
    expect_int("decode_weight_result_gross", (int)weight_result.gross_weight_mg, 0x12345678);
    expect_int("decode_weight_result_net", (int)weight_result.net_weight_mg, 10000);
    expect_int("decode_weight_result_raw", (int)weight_result.raw_adc, 0x00112233);
    expect_u16("decode_weight_result_count", weight_result.sample_count, 10U);
    expect_u16("decode_weight_result_window", weight_result.stable_window_mg, 200U);
    expect_u16("decode_weight_result_duration", weight_result.duration_ms, 1500U);
}

/**
 * @brief 验证 FINAL_SORT_RESULT 是 MP157 上传完成后独立下发的最终分拣命令。
 *
 * 返回值：
 *   无返回值；失败时记录断言失败。
 */
static void test_decode_final_sort_result_payload(void)
{
    uint8_t payload[BINARY_PROTOCOL_FINAL_SORT_RESULT_PAYLOAD_LENGTH];
    BinaryProtocol_FinalSortResultPayload_t final_sort;
    uint8_t ok;

    (void)memset(payload, 0, sizeof(payload));
    write_u16_le(&payload[0], 0x0055U);
    write_u16_le(&payload[2], 0x0102U);
    payload[4] = 2U;
    payload[5] = 2U;
    payload[6] = 1U;
    payload[7] = 91U;
    write_u16_le(&payload[8], 0x0007U);

    ok = BinaryProtocolService_DecodeFinalSortResult(payload,
                                                     (uint8_t)sizeof(payload),
                                                     &final_sort);

    expect_int("final_sort_command_is_0x33",
               (int)BINARY_PROTOCOL_CMD_FINAL_SORT_RESULT,
               0x33);
    expect_int("decode_final_sort_ok", (int)ok, 1);
    expect_u16("decode_final_sort_cycle", final_sort.cycle_id, 0x0055U);
    expect_u16("decode_final_sort_job", final_sort.job_id, 0x0102U);
    expect_int("decode_final_sort_result", (int)final_sort.final_result, 2);
    expect_int("decode_final_sort_bin", (int)final_sort.final_bin, 2);
    expect_int("decode_final_sort_upload", (int)final_sort.upload_status, 1);
    expect_int("decode_final_sort_confidence", (int)final_sort.final_confidence, 91);
    expect_u16("decode_final_sort_options", final_sort.option_bits, 0x0007U);
}

/**
 * @brief 验证 EVENT_REPORT 执行器运动完成事件的编号和字段布局稳定。
 *
 * 返回值：
 *   无返回值；失败时记录断言失败。
 */
static void test_event_report_actuator_move_contract(void)
{
    uint8_t payload[BINARY_PROTOCOL_EVENT_REPORT_PAYLOAD_LENGTH];

    (void)memset(payload, 0, sizeof(payload));
    write_u16_le(&payload[0], 0x0055U);
    payload[2] = BINARY_PROTOCOL_EVENT_ACTUATOR_MOVE_DONE;
    payload[3] = 3U;
    payload[4] = 2U;
    payload[5] = BINARY_PROTOCOL_FAULT_SOURCE_CAMERA_MOTOR;
    payload[6] = 0x00U;
    payload[7] = 0x00U;
    payload[8] = 0x01U;
    payload[9] = 0x02U;
    write_u16_le(&payload[10], 0x1234U);
    write_u16_le(&payload[12], 0x0000U);
    write_u16_le(&payload[14], 0x0000U);

    expect_int("event_report_length", (int)BINARY_PROTOCOL_EVENT_REPORT_PAYLOAD_LENGTH, 16);
    expect_int("event_actuator_move_done", (int)BINARY_PROTOCOL_EVENT_ACTUATOR_MOVE_DONE, 0x14);
    expect_int("event_actuator_move_timeout", (int)BINARY_PROTOCOL_EVENT_ACTUATOR_MOVE_TIMEOUT, 0x15);
    expect_int("event_actuator_estimated_done_status",
               (int)BINARY_PROTOCOL_ACTUATOR_MOVE_STATUS_ESTIMATED_DONE,
               0x0005);
    expect_int("event_report_payload_code", (int)payload[2], (int)BINARY_PROTOCOL_EVENT_ACTUATOR_MOVE_DONE);
    expect_u16("event_report_related_seq",
               (uint16_t)(payload[10] | ((uint16_t)payload[11] << 8)),
               0x1234U);
}

/**
 * @brief 主机回归测试入口。
 * @return int 0 表示所有协议解码测试通过，1 表示至少一项失败。
 *
 * 主要流程：
 *   1. 依次运行 CRC、帧解析和各类 payload 解码测试；
 *   2. 如果 g_failed_count 非 0，则打印失败数量并返回 1；
 *   3. 全部通过时打印固定成功文本，方便脚本判断。
 */
int main(void)
{
    test_crc_standard_vector();
    test_parse_start_cycle_frame();
    test_parse_crc_error();
    test_decode_pause_resume_payload();
    test_decode_vision_pos_payload();
    test_decode_stepper_param_payload();
    test_decode_weight_calibration_payload();
    test_decode_weight_calibration_rejects_wrong_length();
    test_decode_model_ready_payload();
    test_decode_arm_job_start_payload();
    test_decode_weight_result_payload();
    test_decode_final_sort_result_payload();
    test_event_report_actuator_move_contract();

    if (g_failed_count != 0)
    {
        printf("binary protocol host tests failed: %d\n", g_failed_count);
        return 1;
    }

    printf("binary protocol host tests passed\n");
    return 0;
}
