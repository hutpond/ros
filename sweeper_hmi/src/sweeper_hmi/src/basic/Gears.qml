import QtQuick 2.0

Item {

  Rectangle {
    anchors.fill: parent
    border.width: 1
    border.color: "black"
  }

  Item {
    width: parent.width
    height: parent.height * 0.8
    anchors.centerIn: parent

    property double itemWidth: height * 0.8
    property double spacing: (width - itemWidth * 3) / 4

    Rectangle {
      x: parent.spacing
      width: parent.itemWidth
      height: parent.itemWidth
      anchors.verticalCenter: parent.verticalCenter
      radius: width * 0.5
      color: "white"
      border.color: "gray"
      border.width: 2

      Text {
        anchors.verticalCenter: parent.verticalCenter
        anchors.horizontalCenter: parent.horizontalCenter
        color: "black"
        text: qsTr("D")
        font.pointSize: 18
      }
    }

    Rectangle {
      x: parent.spacing * 2 + parent.itemWidth
      width: parent.itemWidth
      height: parent.itemWidth
      anchors.verticalCenter: parent.verticalCenter
      radius: width * 0.5
      color: "black"

      Text {
        anchors.verticalCenter: parent.verticalCenter
        anchors.horizontalCenter: parent.horizontalCenter
        color: "white"
        text: "N"
        font.pointSize: 18
      }
    }

    Rectangle {
      x: parent.spacing * 3 + parent.itemWidth * 2
      width: parent.itemWidth
      height: parent.itemWidth
      anchors.verticalCenter: parent.verticalCenter
      radius: width * 0.5
      color: "black"

      Text {
        anchors.centerIn: parent.Center
        anchors.verticalCenter: parent.verticalCenter
        anchors.horizontalCenter: parent.horizontalCenter
        color: "white"
        text: qsTr("R")
        font.pointSize: 20
      }
    }
  }
}
