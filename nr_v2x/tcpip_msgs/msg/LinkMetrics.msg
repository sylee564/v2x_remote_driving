# --- 식별/라벨 ---
uint32   socket_id          # 드라이버 내부 map key (소켓 식별)
string   label              # 예: "TCP#3/VIDEO0", socket_t::label()

# --- window metrics (sample_and_reset_window() 결과) ---
float64  tx_bps
float64  rx_bps
uint64   tx_frames_delta
uint64   rx_frames_delta
uint64   rx_missing_delta
uint64   rx_dup_delta
uint64   rx_reorder_delta
float64  pdr_rx            # rx_frames_delta / (rx_frames_delta + rx_missing_delta)

# --- cumulative metrics (총계) ---
uint64   tx_frames
uint64   rx_frames
uint64   rx_missing
uint64   rx_dup
uint64   rx_reorder
uint64   tx_bytes
uint64   rx_bytes

# --- (선택) 타임스탬프/채널 ---
time     stamp              # 드라이버 publish 시각 (header 뜨겁게 안 쓸 때)
uint8    channel            # app_channel_t 값 그대로 (VIDEO0 등)
