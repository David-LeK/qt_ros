#ifndef PATHREFERENCE_H
#define PATHREFERENCE_H

#include <QGeoPath>
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QTimer>
#include <QtCore/qdebug.h>

class PathReference : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QGeoPath geopathref READ geoPathref WRITE setGeoPathref NOTIFY geopathrefChanged)
public:
    PathReference(QObject *parent = nullptr) : QObject(parent) {}
    QGeoPath geoPathref() const { return mGeoPath; }
    void setGeoPathref(const QGeoPath &geoPath)
    {
        if (geoPath == mGeoPath)
            return;
        mGeoPath = geoPath;
        emit geopathrefChanged();
    }

signals:
    void geopathrefChanged();

public:
    QGeoPath mGeoPath;
};

#endif // PATHREFERENCE_H
