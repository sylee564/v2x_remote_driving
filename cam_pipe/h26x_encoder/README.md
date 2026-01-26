# ROS H.26x Video (Encoder/Decoder)

ROS1 Noetic (Ubuntu 20.04)에서 **H.264/H.265** 영상 스트림을 ROS 토픽으로 인/디코딩하는 패키지 모음입니다.

- **h26x_encoder**: `sensor_msgs/Image` → **H.26x(Annex-B, byte-stream)** `EncodedFrame` 퍼블리시  
- **h26x_decoder**: `EncodedFrame` → **sensor_msgs/Image (BGR8)** 복원

Jetson은 **nvv4l2** 하드웨어 코덱을 자동 감지하고, 일반 PC는 **x264/x265 + avdec**(CPU)을 사용합니다.


---

## 📦 Dependency

### ROS
```bash
sudo apt update
sudo apt install -y \
  ros-noetic-ros-base ros-noetic-roscpp ros-noetic-std-msgs ros-noetic-sensor-msgs \
  ros-noetic-cv-bridge ros-noetic-image-view ros-noetic-nodelet
  ros-noetic-camera-info-manager
```
```bash
sudo apt install -y \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
```

## 🔧 Build

### catkin workspace
```bash
cd ~/catkin_ws/src
git clone https://github.com/<your-org>/ros-h26x-video.git
cd ..
catkin_make
source devel/setup.bash
```

## 🚀 Quick Start

### Encoder (Node)
```bash
roslaunch h26x_encoder encoder_front.launch \
  input_topic:=/front/camera/image_raw \
  camera_info_topic:=/front/camera/camera_info \
  codec:=h264 bitrate_bps:=2500000 gop:=30 mode:=auto \
  fps_mode:=nominal fps_nominal:=30 fps_hint:=30
```

### Decoder (Node)
```bash
roslaunch h26x_decoder decoder_node.launch \
  input_topic:=/front/encoded/h264 \
  output_topic:=/front/decoded/image \
  codec:=auto mode:=auto wait_for_idr:=true fps_hint:=30
```

## 🧾 메시지 정의

### h26x_encoder/EncodedFrame.msg
```msg
  std_msgs/Header header
  string  codec              # "h264" | "h265"
  uint32  width
  uint32  height
  uint32  fps_num
  uint32  fps_den
  bool    keyframe           # true if IDR / non-delta
  uint32  target_bitrate     # in bps
  uint8[] data               # Access Unit (Annex-B byte-stream)
```

### h26x_encoder/EncodedFrame.msg
```msg
std_msgs/Header header
float64 fps_measured       # EMA 또는 윈도 평균
float64 fps_instant        # 직전 프레임 간격 역수
float64 jitter_ms_rms      # RMS 지터(ms)
uint32  frames_total
uint32  frames_dropped
```

## 🧩 토픽 & 파라미터
### h26x_encoder

#### Subscribes

input_topic (sensor_msgs/Image, 기본 /front/image_raw)

camera_info_topic (sensor_msgs/CameraInfo, 기본 /front/camera_info)

#### Publishes

output_topic (h26x_encoder/EncodedFrame, 기본 encoded/<codec>)

통계: <output_topic> 기준 …/stats (예: encoded/h264 → encoded/stats) 구현 버전에 따라 고정 토픽일 수 있음.

#### 주요 파라미터

| 이름             |     타입 |               기본값 | 설명                            |
| -------------- | -----: | ----------------: | ----------------------------- |
| `codec`        | string |            `h264` | `h264` 또는 `h265`              |
| `mode`         | string |            `auto` | `auto` / `cpu` / `hw`         |
| `bitrate_bps`  |    int |         `2500000` | 목표 비트레이트(bps)                 |
| `gop`          |    int |              `30` | IDR 주기(프레임)                   |
| `fps_mode`     | string |         `nominal` | `nominal` or `measured`       |
| `fps_nominal`  |    int |              `30` | `fps_mode=nominal`일 때 기록될 FPS |
| `fps_hint`     |    int |              `30` | caps 협상 힌트(FPS)               |
| `output_topic` | string | `encoded/<codec>` | 퍼블리시 토픽                       |

### h26x_decoder

#### Subscribes

input_topic (h26x_encoder/EncodedFrame, 기본 encoded/h264)

#### Publishes

output_topic (sensor_msgs/Image BGR8, 기본 image_decoded)

통계: <output_topic> 기준 …/stats (예: /front/decoded/image → /front/decoded/stats)

#### 주요 파라미터
| 이름             |     타입 |    기본값 | 설명                                    |
| -------------- | -----: | -----: | ------------------------------------- |
| `codec`        | string | `auto` | `auto/h264/h265` (auto는 msg.codec 우선) |
| `mode`         | string | `auto` | `auto/cpu/hw`                         |
| `wait_for_idr` |   bool | `true` | 시작 시 IDR 전까지 드롭                       |
| `fps_hint`     |    int |   `30` | PTS/DURATION 추정 힌트                    |
