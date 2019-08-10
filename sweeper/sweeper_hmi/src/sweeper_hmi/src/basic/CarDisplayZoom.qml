import QtQuick 2.0

Item {

  Image {
    id: direction
    source: "qrc:/svg/direction.svg"

    anchors.horizontalCenter: parent.horizontalCenter
    anchors.top: parent.top
    anchors.topMargin: 50

    rotation: 30
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


}
