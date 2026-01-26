std_msgs/Header header

# 기존
float64 fps_measured
float64 fps_instant
float64 jitter_ms_rms
uint32  frames_total
uint32  frames_dropped

# 추가(운영에 유용)
float64 encode_ms_avg         # 인코더 처리 평균 지연
float64 encode_ms_p95
uint32  bitrate_actual        # 직전 윈도 실제 인코딩 비트레이트(bps)
float64 vbv_fullness_ratio    # 0.0~1.0 (미사용 시 음수/NaN)

float64 e2e_latency_ms        # (선택) 카메라 캡처→디코드(or 표시)까지, 측정 가능할 때
float64 tx_latency_ms         # (선택) 송신 지연
float64 rx_latency_ms         # (선택) 수신/재조립 지연
uint32  frame_crc32_mismatch  # 누적 CRC 불일치 카운터
