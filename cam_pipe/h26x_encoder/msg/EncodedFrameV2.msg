# ===== Enums (string 대신 uint8 상수) =====
uint8 CODEC_H264=1
uint8 CODEC_H265=2

uint8 LAYOUT_ANNEXB=0     # 00 00 01 start-code
uint8 LAYOUT_AVCC=1       # length-prefixed NALU (mp4/FLV 스타일)

uint8 FRAME_TYPE_UNKNOWN=0
uint8 FRAME_TYPE_IDR=1    # H.264 IDR / H.265 IDR
uint8 FRAME_TYPE_I=2
uint8 FRAME_TYPE_P=3
uint8 FRAME_TYPE_B=4

uint8 CHROMA_420=1
uint8 CHROMA_422=2
uint8 CHROMA_444=3

uint8 CLOCK_MONOTONIC=1   # steady_clock 기반(권장)
uint8 CLOCK_SYSTEM=2
uint8 CLOCK_PTP=3
uint8 CLOCK_TAI=4

# ===== Payload =====
std_msgs/Header header      # publish 시각(ROS time). clock_domain으로 어떤 시계를 쓰는지 명시.
uint8   codec               # CODEC_*
uint8   data_layout         # LAYOUT_*  (AnnexB/AVCC 명확화)
uint32  width
uint32  height
uint32  fps_num
uint32  fps_den

uint8   frame_type          # FRAME_TYPE_* (keyframe의 유형까지 식별)
bool    keyframe            # 편의상 유지(= frame_type이 IDR/I면 true)

uint8   stream_id           # 멀티 스트림 구분(0,1,2…)
uint32  frame_id            # 롤오버 허용

# 타임스탬프: 재정렬(B-frame) 및 지터 분석에 필요
uint64  pts_us              # presentation timestamp (μs)
uint64  dts_us              # decode timestamp (μs, B-frame 있으면 pts와 다름; 없으면 pts와 동일)
int32   cts_offset_us       # pts_us - dts_us (표시 편의, 음수 가능성 거의 없음)

uint8   clock_domain        # CLOCK_*

# 인코딩 파라미터(동적)
uint32  target_bitrate      # bps (설정값; 실제는 stats에)

# 색/포맷 메타(디코더 초기화 안정성 ↑)
uint8   chroma_format       # CHROMA_*
uint8   bit_depth_luma      # 8/10/12 …
uint8   bit_depth_chroma
uint16  sar_num             # sample aspect ratio (0이면 미지정)
uint16  sar_den
uint8   color_range         # 0=unspecified, 1=limited(MPEG), 2=full(JPEG)
uint8   color_primaries     # BT.709=1, BT.2020=9 등 (0=unspecified)
uint8   transfer_characteristics
uint8   matrix_coeffs

# 무결성 검증(재조립/전송 오류 탐지)
uint32  frame_crc32         # data에 대한 CRC32(0이면 미사용)

# 코덱 extradata가 필요한 AVCC일 때는 별도 토픽/서비스로 배포 권장
uint8[] data                # 1 AU per message (ANNEX-B 또는 AVCC 규약을 data_layout로 명시)
