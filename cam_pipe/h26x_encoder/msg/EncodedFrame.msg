std_msgs/Header header          # 게시 시각(또는 파이프라인 상의 처리 시각)
string codec                    # "h264" | "h265"
uint32 width
uint32 height
uint32 fps_num                  # 명세 FPS(예: 30/1)
uint32 fps_den
bool   keyframe                 # true = IDR/CRA 등 non-delta AU
uint32 target_bitrate           # bps (CBR 기준)

uint8  stream_id                # 0,1,2... (멀티 카메라/스트림 구분)
uint32 frame_id                 # 프레임 증가값(롤오버 허용)
uint64 pts_us                   # 원본 캡처 PTS(μs, 모노토닉)

uint8[] data                    # Annex-B byte-stream, 1 AU per message
