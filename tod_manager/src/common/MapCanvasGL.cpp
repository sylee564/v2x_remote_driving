#include "tod_manager/MapCanvasGL.h"
#include <QtGlobal>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QtMath>
#include <cmath>
#include <algorithm>

namespace {
inline double deg2rad(double d){ return d * M_PI / 180.0; }
}

// ---------------- ctor & GL lifecycle ----------------
MapCanvasGL::MapCanvasGL(QWidget* parent) : QOpenGLWidget(parent) {
  setMouseTracking(true);
  // painter로만 그리므로 VAO/VBO 없음 → GL 타이밍 이슈 X
}

void MapCanvasGL::initializeGL() {
  // 배경색은 paintGL에서 지움
}

void MapCanvasGL::resizeGL(int, int) {
  updateViewTransform();
}

void MapCanvasGL::paintGL() {
  // 배경
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing, true);
  p.fillRect(rect(), QColor(16,18,22));

  // 맵 드로우
  auto drawBatch = [&](const PolyBatch& batch, const QPen& pen){
    p.setPen(pen);
    for (const auto& pl : batch) {
      if (pl.size() < 2) continue;
      QPolygonF poly;
      poly.reserve(pl.size());
      for (const auto& q : pl) poly << world2screen_.map(q);
      p.drawPolyline(poly);
    }
  };

  drawBatch(left_,   pen_left_);
  drawBatch(right_,  pen_right_);
  drawBatch(center_, pen_center_);

  // 차량
  p.setPen(Qt::NoPen);
  p.setBrush(QColor(60, 220, 100, 220));

  const QPointF c = world2screen_.map(veh_xy_);
  if (fmode_ == FootprintMode::Circle) {
    const double radius_px = 2.0 + 0.25 * pixels_per_meter_; // 대략적 스케일
    p.drawEllipse(c, radius_px, radius_px);
  } else {
    // 직사각 풋프린트(Autoware 느낌)
    const double L = veh_len_;
    const double W = veh_wid_;
    const double RO = veh_ro_; // rear overhang

    // 차량 좌표계 기준 꼭짓점(좌표 원점 = rear bumper 기준 x=0)
    QVector<QPointF> body{
      { RO,            -W*0.5 },
      { RO + L,        -W*0.5 },
      { RO + L,         W*0.5 },
      { RO,             W*0.5 }
    };

    // 월드 변환(회전+평행이동)
    const double cth = std::cos(veh_yaw_), sth = std::sin(veh_yaw_);
    QPolygonF poly_w;
    for (const auto& b : body) {
      QPointF w( veh_xy_.x() + cth*b.x() - sth*b.y(),
                 veh_xy_.y() + sth*b.x() + cth*b.y() );
      poly_w << world2screen_.map(w);
    }
    p.drawPolygon(poly_w);
  }

  // 정보 텍스트
  p.setPen(Qt::white);
  p.drawText(10, 20, QString("Lat0=%.7f  Lon0=%.7f  scale=%.2f px/m")
             .arg(lat0_deg_, 0, 'f', 7).arg(lon0_deg_, 0, 'f', 7).arg(pixels_per_meter_));
}

QPointF MapCanvasGL::screenToWorld(const QPointF& s) const {
  const QTransform inv = world2screen_.inverted();
  return inv.map(s);
}

// ====== 휠 줌 ======
void MapCanvasGL::wheelEvent(QWheelEvent* e) {
  if (width() <= 0 || height() <= 0) { e->accept(); return; }

#if QT_VERSION >= QT_VERSION_CHECK(5, 14, 0)
  const QPointF cursor_screen = e->position();
#else
  const QPointF cursor_screen = e->pos();
#endif

  const QPoint numDeg  = e->angleDelta() / 8;   // 1 step = 15 deg
  const double steps   = (numDeg.y() / 15.0);
  const double factor  = std::pow(1.2, steps);

  // 커서 아래 고정 줌: 줌 전 월드좌표
  const QPointF world_before  = screenToWorld(cursor_screen);

  // 스케일 제한
  const double new_ppm = std::clamp(pixels_per_meter_ * factor, kMinPPM_, kMaxPPM_);
  if (std::abs(new_ppm - pixels_per_meter_) < 1e-6) { e->accept(); return; }
  pixels_per_meter_ = new_ppm;

  // 변환 갱신 후 보정
  updateViewTransform();
  const QPointF world_after = screenToWorld(cursor_screen);
  world_center_ += (world_before - world_after);  // 앵커 고정
  updateViewTransform();

  update();
  e->accept();
}


// ====== 패닝 ======
void MapCanvasGL::mousePressEvent(QMouseEvent* e) {
  if (e->button() == Qt::LeftButton) {
    is_panning_     = true;
    last_mouse_pos_ = e->pos();
    setCursor(Qt::ClosedHandCursor);
    e->accept();
  } else {
    QOpenGLWidget::mousePressEvent(e);
  }
}

void MapCanvasGL::mouseMoveEvent(QMouseEvent* e) {
  if (!is_panning_) { QOpenGLWidget::mouseMoveEvent(e); return; }

  const QPoint delta_px = e->pos() - last_mouse_pos_;
  last_mouse_pos_ = e->pos();

  // 화면 픽셀 이동을 월드(m) 이동으로 환산 (y축은 반전)
  if (pixels_per_meter_ > 1e-9) {
    world_center_.rx() -= delta_px.x() / pixels_per_meter_;
    world_center_.ry() += delta_px.y() / pixels_per_meter_;
    updateViewTransform();
    update();
  }
  e->accept();
}

void MapCanvasGL::mouseReleaseEvent(QMouseEvent* e) {
  if (e->button() == Qt::LeftButton && is_panning_) {
    is_panning_ = false;
    unsetCursor();
    e->accept();
  } else {
    QOpenGLWidget::mouseReleaseEvent(e);
  }
}

// ---------------- public APIs ----------------
bool MapCanvasGL::loadLanelet2(const QString& osmPath,
                               const std::string& geoRef,
                               const lanelet::Origin& origin) {
  try {
    std::shared_ptr<lanelet::LaneletMap> map;

    if (!geoRef.empty()) {
      // Local Cartesian (proj4)
      // proj4에 lat_0/lon_0가 들어있으면 lanelet이 내부에서 사용
      lanelet::Origin lc_origin = origin; // origin이 default면 lanelet 내부 처리
      lanelet::projection::LocalCartesianProjector proj(lc_origin);
      map = lanelet::load(osmPath.toStdString(), proj);

      // ENU 원점이 아직 없으면, origin이 default인 경우 맵 중앙으로 설정
      if (!enu_ready_) {
        if (!origin.isDefault) setENUOrigin(origin.position.lat, origin.position.lon);
        else {
          // 맵 중심 대충 추정
          double sx=0, sy=0; std::size_t n=0;
          for (const auto& ll : map->laneletLayer) {
            for (auto& p : lanelet::utils::to2D(ll.leftBound()))
              { sx += p.basicPoint().x(); sy += p.basicPoint().y(); ++n; }
            for (auto& p : lanelet::utils::to2D(ll.rightBound()))
              { sx += p.basicPoint().x(); sy += p.basicPoint().y(); ++n; }
          }
          if (n>0) { world_center_ = QPointF(sx/n, sy/n); updateViewTransform(); }
        }
      }

    } else {
      // UTM + origin (권장: origin 제공)
      lanelet::projection::UtmProjector proj(origin);
      map = lanelet::load(osmPath.toStdString(), proj);

      if (!enu_ready_) {
        if (!origin.isDefault) setENUOrigin(origin.position.lat, origin.position.lon);
      }
    }

    // CPU 버퍼로 추출
    PolyBatch L, R, C;
    extractLanelets(*map, L, R, C);
    left_ = std::move(L);
    right_ = std::move(R);
    center_ = std::move(C);

    // 간단한 뷰 스케일/센터 추정
    if (!left_.empty() && !left_.front().empty()) {
      world_center_ = left_.front().front();
    }
    updateViewTransform();
    update();
    return true;

  } catch (const std::exception& e) {
    qWarning("MapCanvasGL lanelet load failed: %s", e.what());
    return false;
  }
}

void MapCanvasGL::setENUOrigin(double lat_deg, double lon_deg) {
  lat0_deg_ = lat_deg;
  lon0_deg_ = lon_deg;
  enu_ready_ = true;
}

void MapCanvasGL::setVehiclePoseGeo(double lat_deg, double lon_deg, double yaw_rad) {
  if (!enu_ready_) {
    qWarning("MapCanvasGL: ENU origin not set; pose buffered but may be invalid.");
  }
  veh_xy_ = ll2enu(lat_deg, lon_deg);
  veh_yaw_ = yaw_rad;
  update();
}

void MapCanvasGL::centerOnVehicle() {
  world_center_ = veh_xy_;
  updateViewTransform();
  update();
}

// ---------------- internals ----------------
void MapCanvasGL::extractLanelets(const lanelet::LaneletMap& map,
                                  PolyBatch& left, PolyBatch& right, PolyBatch& center) {
  left.clear(); right.clear(); center.clear();
  left.reserve(map.laneletLayer.size());
  right.reserve(map.laneletLayer.size());
  center.reserve(map.laneletLayer.size());

  for (const auto& ll : map.laneletLayer) {
    // 좌/우 경계
    Polyline lpl, rpl;
    auto L = lanelet::utils::to2D(ll.leftBound());
    auto R = lanelet::utils::to2D(ll.rightBound());
    for (auto& p : L) lpl.push_back(QPointF(p.basicPoint().x(), p.basicPoint().y()));
    for (auto& p : R) rpl.push_back(QPointF(p.basicPoint().x(), p.basicPoint().y()));
    if (!lpl.isEmpty()) left.push_back(std::move(lpl));
    if (!rpl.isEmpty()) right.push_back(std::move(rpl));

    // 센터라인(있으면)
    if (ll.hasCustomCenterline()) {
      Polyline cpl;
      auto C = lanelet::utils::to2D(ll.centerline());
      for (auto& p : C) cpl.push_back(QPointF(p.basicPoint().x(), p.basicPoint().y()));
      if (!cpl.isEmpty()) center.push_back(std::move(cpl));
    }
  }
}

QPointF MapCanvasGL::ll2enu(double lat_deg, double lon_deg) const {
  // 간단한 ENU 근사 (작은 구역에서 충분)
  static constexpr double R = 6378137.0; // WGS84 a
  const double lat0 = deg2rad(lat0_deg_);
  const double x = (deg2rad(lon_deg - lon0_deg_)) * std::cos(lat0) * R;
  const double y = (deg2rad(lat_deg - lat0_deg_)) * R;
  return QPointF(x, y);
}

void MapCanvasGL::updateViewTransform() {
  // 월드(m) → 화면(px) 변환 구성
  // y축 위로 증가(수학 좌표) → 화면에서는 아래가 +y 이므로 y에 -scale 사용
  const double s = pixels_per_meter_;
  world2screen_.reset();
  world2screen_.translate(width()*0.5, height()*0.5);
  world2screen_.scale(s, -s);
  world2screen_.translate(-world_center_.x(), -world_center_.y());
}
