import QtQuick 2.0

Item {

  CarLocation {
    id: carLocation

    x: 0
    y: 0
    width: parent.width * 0.28
    height: parent.height
  }

  CarDisplayZoom {
    id: carDisplayZoom

    x: parent.width - width
    y: 0
    width: parent.width * 0.28
    height: parent.height
  }
}
