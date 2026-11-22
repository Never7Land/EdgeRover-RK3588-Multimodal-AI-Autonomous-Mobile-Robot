std_msgs/Header header

# 检测（上一个统计窗口）
float32 det_hz
float32 det_age_ms        # 窗口内平均；窗口内没收到 det 时为 0
float32 det_age_ms_max
float32 infer_ms          # 窗口内平均
float32 infer_ms_max
uint32  det_count         # 最近一帧的目标数
float32 det_silence_s     # 距上一条 det 的秒数；-1 = 启动后从未收到

# 图像
float32 image_hz

# VLM
bool    vlm_busy
bool    vlm_last_ok
float32 vlm_last_total_ms
float32 vlm_last_rtt_ms
float32 vlm_last_frame_age_ms
uint32  vlm_ok_count
uint32  vlm_err_count