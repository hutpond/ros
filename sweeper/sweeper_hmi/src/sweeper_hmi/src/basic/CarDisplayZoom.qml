import QtQuick 2.0

Item {

  property double yawAngle: 0

  Image {
    id: direction
    source: "qrc:/svg/direction.svg"

    anchors.horizontalCenter: parent.horizontalCenter
    anchors.top: parent.top
    anchors.topMargin: 50

    width: sourceSize.width * 0.5
    height: sourceSize.height * 0.5
    scale: Qt.KeepAspectRatio

    rotation: yawAngle
  }

  Image {
    id: zoomIn
    source: "qrc:/svg/zoomIn.svg"

    anchors.horizontalCenter: parent.horizontalCenter
    anchors.bottom: zoomOut.top
    anchors.bottomMargin: 50
    height: width
  }

  Image {
    id: zoomOut
    source: "qrc:/svg/zoomOut.svg"

    anchors.horizontalCenter: parent.horizontalCenter
    anchors.bottom: parent.bottom
    anchors.bottomMargin: 50
    height: width
  }

  function setYawAngle(yaw) {
    if (direction.visible && Math.abs(yaw - yawAngle) > 1.0) {
      yawAngle = yaw;
      direction.update()
    }
  }
}
