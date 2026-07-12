#ifndef ARTEMIS_CONFIG_H
#define ARTEMIS_CONFIG_H

/* 普通循迹 PID 模式编号：使用固定 Kp/Ki/Kd 计算循迹转向。 */
#define ARTEMIS_LINE_PID_ORIGINAL 0
/* 模糊循迹 PID 模式编号：根据误差大小和误差变化量动态调整 Kp/Ki/Kd。 */
#define ARTEMIS_LINE_PID_FUZZY 1
#ifndef ARTEMIS_LINE_PID_MODE
/* 当前循迹 PID 模式。改成 ARTEMIS_LINE_PID_FUZZY 即可启用模糊 PID。 */
#define ARTEMIS_LINE_PID_MODE ARTEMIS_LINE_PID_ORIGINAL
#endif
/* 航向 PID 的比例项：调大后朝目标角修正更积极，过大容易左右摆动。 */
#define ARTEMIS_YAW_PID_KP 0.3f
/* 航向 PID 的积分项：用于消除长期偏差；当前默认不用积分，避免慢性累积导致过冲。 */
#define ARTEMIS_YAW_PID_KI 0.0f
/* 航向 PID 的微分项：抑制 yaw 误差变化过快，调大后更稳但响应会变钝。 */
#define ARTEMIS_YAW_PID_KD 0.015f
/* 航向误差滤波中上一帧误差的权重；调大后更平滑，但反应更慢。 */
#define ARTEMIS_YAW_ERROR_FILTER_PREVIOUS 0.3f
/* 航向误差滤波中当前误差的权重；通常与 PREVIOUS 相加为 1。 */
#define ARTEMIS_YAW_ERROR_FILTER_CURRENT 0.7f
/* 航向 PID 输出缩放因子；调大后实际转向量变小，调小后转向更猛烈。 */
#define ARTEMIS_YAW_TURN_SCALE 4.5f
/* 认为航向已经稳定的误差阈值，单位为度。 */
#define ARTEMIS_YAW_STABLE_ERROR_DEG 2.0f
/* 航向误差连续小于阈值多少帧后，认为转向稳定。 */
#define ARTEMIS_YAW_STABLE_FRAMES 10U
/* 普通循迹 PID 的比例项：调大后压线修正更快，过大容易蛇形摆动。 */
#define ARTEMIS_LINE_ORIGINAL_KP 25.0f
/* 普通循迹 PID 的积分项：用于消除长期偏差；默认 0，现场需要时再小幅开启。 */
#define ARTEMIS_LINE_ORIGINAL_KI 0.0f
/* 普通循迹 PID 的微分项：抑制误差快速变化，调大后能减小过冲但可能变迟钝。 */
#define ARTEMIS_LINE_ORIGINAL_KD 3.5f
/* 循迹输出基础缩放因子；调大后转向更小，调小后转向更强。 */
#define ARTEMIS_LINE_OUTPUT_BASE_SCALE 150.0f
/* 模糊 PID 中线误差归一化分母；调小会让误差更快进入“大误差”状态。 */
#define ARTEMIS_LINE_FUZZY_ERROR_NORMALIZER 4.0f
/* 模糊 PID 中误差变化量归一化分母；调小会让变化量更快进入“剧烈变化”状态。 */
#define ARTEMIS_LINE_FUZZY_DELTA_NORMALIZER 4.0f
/* 模糊 PID 中“接近中心”的判定宽度；影响 Ki 是否参与补偿。 */
#define ARTEMIS_LINE_FUZZY_CENTER_WIDTH 2.0f
/* 模糊 PID 中“误差变化平稳”的判定宽度；影响 Ki 是否参与补偿。 */
#define ARTEMIS_LINE_FUZZY_STEADY_WIDTH 2.0f
/* 模糊 PID 的 Kp 基础值：无论误差大小，比例项至少有这个强度。 */
#define ARTEMIS_LINE_FUZZY_KP_BASE 18.0f
/* 模糊 PID 中误差大小对 Kp 的增益；调大后偏离中心时修正更强。 */
#define ARTEMIS_LINE_FUZZY_KP_ERROR_GAIN 20.0f
/* 模糊 PID 中误差变化量对 Kp 的增益；调大后快速偏移时比例修正更强。 */
#define ARTEMIS_LINE_FUZZY_KP_DELTA_GAIN 4.0f
/* 模糊 PID 的 Kd 基础值：无论误差大小，微分项至少有这个阻尼。 */
#define ARTEMIS_LINE_FUZZY_KD_BASE 1.5f
/* 模糊 PID 中误差大小对 Kd 的增益；调大后大偏差时更抑制过冲。 */
#define ARTEMIS_LINE_FUZZY_KD_ERROR_GAIN 2.0f
/* 模糊 PID 中误差变化量对 Kd 的增益；调大后变化剧烈时阻尼更强。 */
#define ARTEMIS_LINE_FUZZY_KD_DELTA_GAIN 8.0f
/* 模糊 PID 的 Ki 最大基础值；只有误差小且变化平稳时才会接近该值。 */
#define ARTEMIS_LINE_FUZZY_KI_BASE 0.02f
/* 模糊 PID 的积分限幅，防止积分项长时间累积后突然造成大转向。 */
#define ARTEMIS_LINE_FUZZY_INTEGRAL_LIMIT 32.0f
/* 编码器标定值：多少 tick 等于 1 cm，用于把桥接反馈的累计 tick 换算成距离。 */
#define ARTEMIS_ENCODER_TICKS_PER_CM 10.62f
/* START 命令请求的仿真最大运行时间，单位为秒。 */
#define ARTEMIS_START_MAX_TIME_S 120.0f
/* 仿真控制周期，单位为秒；当前桥接和任务按 20 ms 一帧设计。 */
#define ARTEMIS_CONTROL_PERIOD_S 0.02f
/* 仿真初始进度索引；默认从任务路径起点开始。 */
#define ARTEMIS_INITIAL_PROGRESS_INDEX 0U
/* 任务重复圈数：0 表示无限循环；1 表示按原 task3 只跑一遍后 STOP。 */
#ifndef ARTEMIS_MISSION_REPEAT_COUNT
#define ARTEMIS_MISSION_REPEAT_COUNT 0U
#endif
/* 正式任务阶段等待桥接响应的超时时间，单位 ms。 */
#define ARTEMIS_RESPONSE_TIMEOUT_MS 6000U
/* START 握手阶段等待 STARTED 的超时时间，单位 ms；较短可让桥接后启动时更快握手。 */
#define ARTEMIS_START_RESPONSE_TIMEOUT_MS 1000U
/* 握手失败或重置后再次发送 START 前的延迟，单位 ms。 */
#define ARTEMIS_RETRY_DELAY_MS 1000U
/* 找线阶段连续看到黑线多少帧后，确认已经接触黑线。 */
#define ARTEMIS_LINE_CONFIRM_FRAMES 2U
/* 循迹阶段连续多少帧全 0 后，确认已经丢线并结束循迹动作。 */
#define ARTEMIS_LINE_LOSS_FRAMES 50U
/* PA14 指示灯检测接触/离开黑线时，需要连续确认的帧数。 */
#define ARTEMIS_LED_CONFIRM_FRAMES 2U
/* PA14 指示灯每次闪烁点亮时长，单位 ms。 */
#define ARTEMIS_LED_ON_MS 500U
/* 多个闪烁事件排队时，两次亮灯之间的灭灯间隔，单位 ms。 */
#define ARTEMIS_LED_GAP_MS 100U
/* UART RX 原始字节环形缓冲大小；必须是 2 的幂。 */
#define ARTEMIS_UART_RX_BUFFER_SIZE 512U
/* UART 单行文本最大长度，用于 STARTED/OBS/FINISHED/ERR 分帧。 */
#define ARTEMIS_UART_LINE_BUFFER_SIZE 256U
/* UART TX 格式化缓冲大小，用于 START/STEP/STOP 输出。 */
#define ARTEMIS_UART_TX_BUFFER_SIZE 128U

#if (ARTEMIS_LINE_PID_MODE != ARTEMIS_LINE_PID_ORIGINAL) && \
    (ARTEMIS_LINE_PID_MODE != ARTEMIS_LINE_PID_FUZZY)
#error "ARTEMIS_LINE_PID_MODE must be ORIGINAL or FUZZY"
#endif

#endif
