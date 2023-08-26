#ifndef PATHCONTROLLER_H
#define PATHCONTROLLER_H

#include <QGeoPath>
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QTimer>
#include <QtCore/qdebug.h>

class PathController : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QGeoPath geopath READ geoPath WRITE setGeoPath NOTIFY geopathChanged)
    Q_PROPERTY(float rotation MEMBER rot)
public:
    float rot = 0;
    PathController(QObject *parent = nullptr) : QObject(parent) {}
    QGeoPath geoPath() const { return mGeoPath; }
    void setGeoPath(const QGeoPath &geoPath)
    {
        if (geoPath == mGeoPath)
            return;
        mGeoPath = geoPath;
        emit geopathChanged();
    }

signals:
    void geopathChanged();

public:
    QGeoPath mGeoPath;
};

#endif // PATHCONTROLLER_H
