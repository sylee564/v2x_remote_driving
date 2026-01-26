#include "overlay_pip_nodelet/overlay_pip_core.hpp"

namespace overlay_pip_nodelet {

void OverlayPipCore::init(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
  // -------- 파라미터 로드 --------
  pnh.param("ns",       name_space_,       name_space_);
  pnh.param("pip_scale_w",       pip_scale_w_,       pip_scale_w_);
  pnh.param("pip_scale_h",       pip_scale_h_,       pip_scale_h_);
  pnh.param("pip_margin",        pip_margin_,        pip_margin_);
  pnh.param("queue_size",        queue_size_,        queue_size_);
  pnh.param("bg_dim_alpha",     bg_dim_alpha_,     bg_dim_alpha_);
  pnh.param("overlay_alpha",    overlay_alpha_,    overlay_alpha_);
  pnh.param("draw_border",       draw_border_,       draw_border_);
  pnh.param("border_thickness",  border_thickness_,  border_thickness_);
  pnh.param("control_topic",     control_topic_,     control_topic_);


  // 유효 범위 클램프
  pip_scale_w_ = std::max(0.0, std::min(1.0, pip_scale_w_));
  pip_scale_h_ = std::max(0.0, std::min(1.0, pip_scale_h_));
  border_thickness_ = std::max(0, border_thickness_);
  queue_size_ = std::max(1, queue_size_);

  it_ = std::make_unique<image_transport::ImageTransport>(nh);
  image_transport::TransportHints hints("raw", ros::TransportHints().tcpNoDelay());

  // Rectified 컬러 입력 두 개를 가정: main_image, sub_image
  sub_main_ = it_->subscribe("main_image", queue_size_, &OverlayPipCore::cbMainImage, this, hints);
  sub_sub_  = it_->subscribe("sub_image",  queue_size_, &OverlayPipCore::cbSubImage,  this, hints);
  pub_out_   = it_->advertise("output_image", queue_size_);

  if(name_space_ == "front")
    sub_parking_  = it_->subscribe("parking_image",  queue_size_, &OverlayPipCore::cbParkingImage,  this, hints);

  // 기어 상태 구독
  sub_control_ = nh.subscribe(control_topic_, 1, &OverlayPipCore::cbControlCmd, this);

  ROS_INFO_STREAM("[overlay_pip] init: "
                  << "scaleW=" << pip_scale_w_ << ", scaleH=" << pip_scale_h_
                  << ", margin=" << pip_margin_
                  << ", q=" << queue_size_ << ", border=" << (draw_border_ ? "on" : "off")
                  << ", control_topic=" << control_topic_ );
}

void OverlayPipCore::cbControlCmd(const tod_msgs::ControlCmdConstPtr& msg) {
  // 차량 메시지 정의에 맞게 수정 가능
  gear_state_.store(msg->shift.data, std::memory_order_relaxed);
  indicator_state_.store(msg->indicator.data, std::memory_order_relaxed);
  parking_state_.store(msg->parking_flag, std::memory_order_relaxed);
}

void OverlayPipCore::cbParkingImage(const sensor_msgs::ImageConstPtr& msg) {
  try {
    cv_bridge::CvImageConstPtr cvp = cv_bridge::toCvShare(msg, "bgr8");
    std::lock_guard<std::mutex> lk(mtx_parking_);
    parking_latest_ = cvp->image.clone();       // 최신 한 장만 유지
    parking_latest_header_ = msg->header;
  } catch (const cv_bridge::Exception& e) {
    ROS_WARN_STREAM("[overlay_pip] parking cv_bridge: " << e.what());
  }
}

void OverlayPipCore::cbSubImage(const sensor_msgs::ImageConstPtr& msg) {
  try {
    cv_bridge::CvImageConstPtr cvp = cv_bridge::toCvShare(msg, "bgr8");
    std::lock_guard<std::mutex> lk(mtx_sub_);
    sub_latest_ = cvp->image.clone();       // 최신 한 장만 유지
    sub_latest_header_ = msg->header;
  } catch (const cv_bridge::Exception& e) {
    ROS_WARN_STREAM("[overlay_pip] sub cv_bridge: " << e.what());
  }
}

void OverlayPipCore::cbMainImage(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImageConstPtr cvpMain;
  try {
    cvpMain = cv_bridge::toCvShare(msg, "bgr8"); // main rect color
  } catch (const cv_bridge::Exception& e) {
    ROS_WARN_STREAM("[overlay_pip] main cv_bridge: " << e.what());
    return;
  }

  // sub 최신 프레임 복사
  cv::Mat sub_copy;
  std_msgs::Header sub_hdr_copy;
  {
    std::lock_guard<std::mutex> lk(mtx_sub_);
    if (!sub_latest_.empty()) {
      sub_copy = sub_latest_.clone();
      sub_hdr_copy = sub_latest_header_;
    }
  }

  const uint8_t gear = gear_state_.load(std::memory_order_relaxed);
  const uint8_t ind  = indicator_state_.load(std::memory_order_relaxed);
  const bool parking  = parking_state_.load(std::memory_order_relaxed);

  cv::Mat canvas;
  std_msgs::Header base_header = msg->header; // 기본: main 헤더

  if(name_space_ == "front"){
    const bool is_reverse = (gear == eGearPosition::GEARPOSITION_REVERSE);

    if (is_reverse && !sub_copy.empty()) {
      if(parking){
        cv::Mat parking_copy;
        std_msgs::Header park_hdr_copy;
        {
          std::lock_guard<std::mutex> lk(mtx_sub_);
          if (!sub_latest_.empty()) {
            parking_copy = parking_latest_.clone();
            park_hdr_copy = parking_latest_header_;
          }
        }
        canvas = sub_copy.clone();
        base_header = park_hdr_copy;
        overlayAt(canvas, cvpMain->image, OverlayPos::TopCenter);
      }
      else{
        canvas = sub_copy.clone();
        base_header = sub_hdr_copy;
        overlayAt(canvas, cvpMain->image, OverlayPos::TopCenter);
      }

    } else {
      // Main이 메인, Sub를 상단 중앙에
      canvas = cvpMain->image.clone();
      if (!sub_copy.empty()) {
        overlayAt(canvas, sub_copy, OverlayPos::TopCenter);
      }
    }

  }
  else if(name_space_ == "left"){
    const bool show_side_as_main = (ind == eIndicator::INDICATOR_LEFT) && !sub_copy.empty();

    // 항상 main을 메인 캔버스로 사용
    canvas = cvpMain->image.clone();
    base_header = msg->header;

    if (show_side_as_main) {
      // 좌측 방향지시등 ON인 경우에만 sub를 오른쪽 하단에 PiP로 오버레이
      overlayAt(canvas, sub_copy, OverlayPos::BottomLeft);
    }
  }
  else if(name_space_ == "right"){
    const bool show_side_as_main = (ind == eIndicator::INDICATOR_RIGHT) && !sub_copy.empty();

    // 항상 main을 메인 캔버스로 사용
    canvas = cvpMain->image.clone();
    base_header = msg->header;

    if (show_side_as_main) {
      // 좌측 방향지시등 ON인 경우에만 sub를 오른쪽 하단에 PiP로 오버레이
      overlayAt(canvas, sub_copy, OverlayPos::BottomRight);
    }
  }


  cv_bridge::CvImage out;
  out.header   = base_header;                // 메인 소스 헤더 유지
  out.encoding = "bgr8";           // 보통 "bgr8"
  out.image    = canvas;
  pub_out_.publish(out.toImageMsg());
}
inline cv::Rect OverlayPipCore::makeRect_(int W, int H, int w, int h, OverlayPos pos) const {
  int x=0, y=0;
  switch (pos) {
    case OverlayPos::TopCenter:
      x = std::max(0, (W - w) / 2);
      y = std::max(0, pip_margin_);
      break;
    case OverlayPos::BottomRight:
      x = std::max(0, W - w - pip_margin_);
      y = std::max(0, H - h - pip_margin_);
      break;
    case OverlayPos::BottomLeft:
      x = std::max(0, pip_margin_);
      y = std::max(0, H - h - pip_margin_);
      break;
  }
  // 클램프
  x = std::min(x, std::max(0, W - w));
  y = std::min(y, std::max(0, H - h));
  return {x, y, w, h};
}

void OverlayPipCore::overlayAt(cv::Mat& canvas_bgr, const cv::Mat& pip_bgr, OverlayPos pos) {
  // 1) 필요 시 백미러(좌우 반전)
  cv::Mat pip_src;
  if(name_space_ == "front"){
    if (gear_state_.load(std::memory_order_relaxed) != eGearPosition::GEARPOSITION_REVERSE) 
      cv::flip(pip_bgr, pip_src, 1);  
    else{
      cv::flip(canvas_bgr, canvas_bgr, 1);  
      pip_src = pip_bgr;
    }                
  }
  else{
    pip_src = pip_bgr;

  }


  // 2) 캔버스 비례 목표 크기
  int target_w = std::max(1, static_cast<int>(std::round(canvas_bgr.cols * pip_scale_w_)));
  int target_h = std::max(1, static_cast<int>(std::round(canvas_bgr.rows * pip_scale_h_)));
  target_w = std::min(target_w, canvas_bgr.cols);
  target_h = std::min(target_h, canvas_bgr.rows);

  static thread_local cv::Mat pip_resized;
  cv::resize(pip_src, pip_resized, cv::Size(target_w, target_h), 0, 0, cv::INTER_AREA);

  // 3) 위치 계산
  cv::Rect roi_rect = makeRect_(canvas_bgr.cols, canvas_bgr.rows, target_w, target_h, pos);
  if (roi_rect.x < 0 || roi_rect.y < 0 ||
      roi_rect.br().x > canvas_bgr.cols || roi_rect.br().y > canvas_bgr.rows) {
    ROS_WARN_THROTTLE(1.0, "[overlay_pip] PiP ROI out of canvas");
    return;
  }
  cv::Mat roi = canvas_bgr(roi_rect);

  // 4) 배경 어둡게
  if (bg_dim_alpha_ > 0.0) {
    cv::Mat black(roi.size(), roi.type(), cv::Scalar(0,0,0));
    cv::addWeighted(black, bg_dim_alpha_, roi, 1.0 - bg_dim_alpha_, 0.0, roi);
  }

  // 5) 합성
  if (overlay_alpha_ >= 1.0) {
    pip_resized.copyTo(roi);
  } else if (overlay_alpha_ > 0.0) {
    cv::addWeighted(pip_resized, overlay_alpha_, roi, 1.0 - overlay_alpha_, 0.0, roi);
  }

  // 6) 테두리
  if (draw_border_ && border_thickness_ > 0) {
    cv::rectangle(canvas_bgr, roi_rect, cv::Scalar(200,200,200),
                  border_thickness_, cv::LINE_AA);
  }
}

} // namespace overlay_pip_nodelet
