#pragma once
#include <QObject>


class MapBridge : public QObject {
    Q_OBJECT
public:
    using QObject::QObject;

signals:
    // C++ -> JS
    void setRobotPosition(double lon, double lat, double headingDeg = 0.0, const QString& id = "");
    void setCarPosition(double lon, double lat, double headingDeg = 0.0, const QString& id = "");
    void panTo(double lon, double lat, double zoom);

public slots:
    // JS -> C++
    void onMapClicked(double lon, double lat) {
        qInfo("Map clicked: %.6f, %.6f", lat, lon);
        emit panTo(lon, lat, 16.0);
    }
};
