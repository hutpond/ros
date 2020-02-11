import QtQuick 2.0

Item {

  property int gearMode: 0
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
      color: gearMode == 0 ? "#09A5FF" : "#0066A1"
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
      color: gearMode == 1 ? "#09A5FF" : "#0066A1"
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
      color: gearMode == 2 ? "#09A5FF" : "#0066A1"
      text: qsTr("R")
      font.pointSize: 20
    }
  }

  function setGearMode(value) {
    if (gearD.visible && gearMode !== value) {
      gearMode = value
      gearD.update()
      gearN.update()
      gearR.update()
    }
  }
}
