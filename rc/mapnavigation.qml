import QtQuick 2.9
import QtQuick.Controls 2.0

import QtLocation 5.6
import QtPositioning 5.6

Item {
    visible: true
    width: 1000
    height: 800

    Plugin {
        id: googlemaps
        name: "googlemaps"
        PluginParameter {
                        name:"googlemaps.maps.apikey"
                        value:"[GOOLE_API_KEY]"
                    }
    }

    Location {
            id: mapCentre
            coordinate {
                latitude: 10.760524
                longitude: 106.662392
            }
        }

    MapQuickItem {
        id: marker
        anchorPoint.x: image.width/4
        anchorPoint.y: image.height
        sourceItem: Image {
            id: image
            width: 25; height: 20
            source: "file::/qrc/vessel.png"
        }
        coordinate {
                            latitude: 11
                            longitude: 106
                   }
        rotation: 0
    }

    function recenter(lat,lng) {
               mapCentre.coordinate.latitude = lat
               mapCentre.coordinate.longitude = lng
             }
    function addMarker(latitude, longitude, rotation)
        {
        marker.coordinate.latitude = latitude
        marker.coordinate.longitude = longitude
        marker.rotation = rotation
        myMap.addMapItem(marker)
        }

    Map {
        id: myMap
        anchors.fill: parent
        plugin: googlemaps
        activeMapType: supportedMapTypes[1]
        //center: QtPositioning.coordinate(latitude, longitude)
        center: QtPositioning.coordinate(mapCentre.coordinate.latitude, mapCentre.coordinate.longitude)
        zoomLevel: 50
        copyrightsVisible : false

        MapPolyline {
            id: pl
            line.width: 5
            line.color: 'red'
        }

        //intended map
        MapPolyline {
            id: intend
            line.width: 5
            line.color: 'green'
        }
    }

    function loadPath(){
        var lines = []
        for(var i=0; i < pathController.geopath.size(); i++){
            lines[i] = pathController.geopath.coordinateAt(i)
            recenter(lines[i].latitude, lines[i].longitude)
            addMarker(lines[i].latitude, lines[i].longitude, pathController.rotation)
        }
        return lines;
    }

    function loadPathref(){
        var lines = []
        for(var i=0; i < refpathController.geopathref.size(); i++){
            lines[i] = refpathController.geopathref.coordinateAt(i)
        }
        return lines;
    }

    Connections{
        target: pathController
        onGeopathChanged: {pl.path = loadPath();}
    }

    Connections{
        target: refpathController
        onGeopathrefChanged: {intend.path = loadPathref();}
    }

    Component.onCompleted: pl.path = loadPath()
}
