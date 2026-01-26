#pragma once
#include <QOpenGLWidget>
#include <QPainter>
#include <QVector>
#include <QPointF>
#include <QMatrix4x4>
#include <optional>
#include <vector>
#include <string>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/utility/Utilities.h>
#include <lanelet2_projection/LocalCartesian.h>
#include <lanelet2_projection/UTM.h>

class MapCanvasGL : public QOpenGLWidget {
  Q_OBJECT
public:
  explicit MapCanvasGL(QWidget* parent = nullptr);

  // Lanelet2 로드: proj4 있으면 LocalCartesian, 없으면 UTM+origin
  bool loadLanelet2(const QString& osmPath,
                    const std::string& geoRef,
                    const lanelet::Origin& origin);

  // ENU 원점(초기 포즈/지도 중심 기준)
  void setENUOrigin(double lat_deg, double lon_deg);

  // 차량 그리기 모드(사각형 풋프린트/원 아이콘)
  enum class FootprintMode { Rect, Circle };
  void setFootprintMode(FootprintMode m) { fmode_ = m; update(); }
  void setVehicleRect(double length_m, double width_m, double rear_overhang_m) {
    veh_len_ = length_m; veh_wid_ = width_m; veh_ro_ = rear_overhang_m; update();
  }

  // 차량 포즈(위경도 입력 → 내부 ENU로 변환)
  void setVehiclePoseGeo(double lat_deg, double lon_deg, double yaw_rad = 0.0);
  void centerOnVehicle();

protected:
  void initializeGL() override;
  void resizeGL(int w, int h) override;
  void paintGL() override;

  void wheelEvent(QWheelEvent* e) override;
  void mousePressEvent(QMouseEvent* e) override;
  void mouseMoveEvent(QMouseEvent* e) override;
  void mouseReleaseEvent(QMouseEvent* e) override;

private:
  using Polyline = QVector<QPointF>;          // world meter 좌표
  using PolyBatch = std::vector<Polyline>;    // 여러 라인들의 묶음

  // lanelet 맵을 world 좌표들로 추출
  static void extractLanelets(const lanelet::LaneletMap& map,
                              PolyBatch& left, PolyBatch& right, PolyBatch& center);

  // 위경도→ENU 근사(지역 작은 범위에서 충분)
  QPointF ll2enu(double lat_deg, double lon_deg) const;

  // 화면 변환(m/world) 설정
  void updateViewTransform();
  QPointF screenToWorld(const QPointF& s) const;


private:
  // Lanelet 맵 CPU 버퍼(미터 단위)
  PolyBatch left_, right_, center_;

  // ENU 원점
  double lat0_deg_ = 0.0;
  double lon0_deg_ = 0.0;
  bool   enu_ready_ = false;

  // 차량 상태
  FootprintMode fmode_ = FootprintMode::Rect;
  double veh_len_ = 4.5;     // m
  double veh_wid_ = 1.8;     // m
  double veh_ro_  = 1.0;     // m (뒤 오버행)
  QPointF veh_xy_{0,0};      // ENU meters
  double  veh_yaw_ = 0.0;    // rad

  // 뷰 파라미터
  QPointF world_center_{0,0};   // meter
  double  pixels_per_meter_ = 0.8; // 확대/축소 (px/m)
  QTransform world2screen_;       // Qt 2D 변환(월드→화면)

  // 스타일
  QPen pen_left_{QColor(0,120,255), 2};
  QPen pen_right_{QColor(220,60,60), 2};
  QPen pen_center_{Qt::gray, 1, Qt::DashLine};

  QPoint  last_mouse_pos_{};
  bool    is_panning_{false};

  // ✅ 줌 한계
  const double kMinPPM_ = 0.05;   // px/m
  const double kMaxPPM_ = 20.0;   // px/m
};
