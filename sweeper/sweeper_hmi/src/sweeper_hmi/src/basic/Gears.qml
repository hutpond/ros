import QtQuick 2.0

Item {

  property double itemWidth: Math.min(height, width / 3.0)

  Rectangle {

    id: gearD

    anchors.left: parent.left
    anchors.verticalCenter: parent.verticalCenter

    width: itemWidth
    height: itemWidth

    color: "#163465"

    Text {
      anchors.verticalCenter: parent.verticalCenter
      anchors.horizontalCenter: parent.horizontalCenter
      color: "#09A5FF"
      text: qsTr("D")
      font.pointSize: 18
    }
  }

  Rectangle {
    id: gearN

    anchors.horizontalCenter: parent.horizontalCenter
    anchors.verticalCenter: parent.verticalCenter

    width: itemWidth
    height: itemWidth

    color: "#163465"

    Text {
      anchors.verticalCenter: parent.verticalCenter
      anchors.horizontalCenter: parent.horizontalCenter
      color: "#0066A1"
      text: "N"
      font.pointSize: 18
    }
  }

  Rectangle {
    id: gearR

    anchors.right: parent.right
    anchors.verticalCenter: parent.verticalCenter

    width: itemWidth
    height: itemWidth

    color: "#163465"

    Text {
      anchors.centerIn: parent.Center
      anchors.verticalCenter: parent.verticalCenter
      anchors.horizontalCenter: parent.horizontalCenter
      color: "#0066A1"
      text: qsTr("R")
      font.pointSize: 20
    }
  }
}
