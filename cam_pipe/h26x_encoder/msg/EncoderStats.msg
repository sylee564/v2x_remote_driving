std_msgs/Header header
float64 fps_measured        # 최근 윈도 평균 또는 EMA(권장) FPS
float64 fps_instant         # 직전 두 프레임 간격 기반 즉시 FPS
float64 jitter_ms_rms       # 프레임 간격의 RMS 지터(ms)
uint32  frames_total        # 누적 인코딩/발행된 프레임 수
uint32  frames_dropped      # 인코더 내부 드롭(큐 초과 등)