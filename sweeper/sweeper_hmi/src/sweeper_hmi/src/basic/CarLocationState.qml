import QtQuick 2.0

Item {

  CarLocation {
    id: carLocation

    anchors.left: parent.left
    anchors.top: parent.top
    anchors.bottom: parent.bottom
    width: parent.width * 0.25
  }

  Image {
    id: leftSpace
    source: "qrc:/image/space.png"

    anchors.top: parent.top
    anchors.bottom: parent.bottom
    anchors.left: carLocation.right
    width: 8
  }

  VehicleState {
    id: vehicleState

    anchors.top: parent.top
    anchors.bottom: parent.bottom
    anchors.left: leftSpace.right
    anchors.right: rightSpace.left
  }

  Image {
    id: rightSpace
    source: "qrc:/image/space.png"

    anchors.top: parent.top
    anchors.bottom: parent.bottom
    anchors.right: carDisplayZoom.left
    width: 8
  }

  CarDisplayZoom {
    id: carDisplayZoom

    anchors.right: parent.right
    anchors.top: parent.top
    anchors.bottom: parent.bottom
    width: parent.width * 0.25
  }

  function setPosition(lon, lat) {
    carLocation.setPosition(lon, lat)
  }

  function setYawAngle(yaw) {
    carDisplayZoom.setYawAngle(yaw)
  }
}
