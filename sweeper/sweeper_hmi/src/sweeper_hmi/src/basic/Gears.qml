import QtQuick 2.0

Item {

  property double itemWidth: height * 0.8
  property double spacing: (width - itemWidth * 3) / 4

  Rectangle {

    id: gearD

    anchors.left: parent.left
    anchors.leftMargin: 20
    anchors.verticalCenter: parent.verticalCenter

    width: 35
    height: 35

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

    anchors.left: gearD.right
    anchors.leftMargin: 20
    anchors.verticalCenter: parent.verticalCenter

    width: 35
    height: 35

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

    anchors.left: gearN.right
    anchors.leftMargin: 20
    anchors.verticalCenter: parent.verticalCenter

    width: 35
    height: 35

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
