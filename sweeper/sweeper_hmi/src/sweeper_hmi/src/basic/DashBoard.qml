import QtQuick 2.0
import QtQuick.Layouts 1.3

Item {

  Row {
    id: firstRow

    x: 0
    y: 0
    width: parent.width * 0.8
    height: parent.height / 3
    spacing: 20

    property double itemWidth: Math.min((width - spacing * 6) / 5, height)

    Image {
      width: parent.itemWidth
      height: parent.itemWidth
      anchors.verticalCenter: parent.verticalCenter

      source: "qrc:/image/garbage.png"
    }

    Image {
      id: water
      width: parent.itemWidth
      height: parent.itemWidth
      anchors.verticalCenter: parent.verticalCenter

      source: "qrc:/image/water.png"
    }

    Image {
      id: brake
      width: parent.itemWidth
      height: parent.itemWidth
      anchors.verticalCenter: parent.verticalCenter

      source: "qrc:/image/brake.png"
    }

    Image {
      id: fault
      width: parent.itemWidth
      height: parent.itemWidth
      anchors.verticalCenter: parent.verticalCenter

      source: "qrc:/image/fault.png"
    }

    Image {
      id: turnsigal
      width: parent.itemWidth
      height: parent.itemWidth
      anchors.verticalCenter: parent.verticalCenter

      source: "qrc:/image/turnsigal.png"
    }
  }

  Row {
    id: secondRow

    x: 0
    y: parent.height / 3
    width: parent.width * 0.8
    height: parent.height / 3
    spacing: 20

    property double itemWidth: Math.min((width - spacing * 6) / 5, height)

    Image {
      id: pallet
      width: parent.itemWidth
      height: parent.itemWidth
      anchors.verticalCenter: parent.verticalCenter

      source: "qrc:/image/pallet.png"
    }

    Image {
      id: spout
      width: parent.itemWidth
      height: parent.itemWidth
      anchors.verticalCenter: parent.verticalCenter

      source: "qrc:/image/spout.png"
    }

    Image {
      id: positionLamp
      width: parent.itemWidth
      height: parent.itemWidth
      anchors.verticalCenter: parent.verticalCenter

      source: "qrc:/image/positionLamp.png"
    }

    Image {
      id: highBeam
      width: parent.itemWidth
      height: parent.itemWidth
      anchors.verticalCenter: parent.verticalCenter

      source: "qrc:/image/highBeam.png"
    }

    Image {
      id: dippedLight
      width: parent.itemWidth
      height: parent.itemWidth
      anchors.verticalCenter: parent.verticalCenter

      source: "qrc:/image/dippedLight.png"
    }
  }

  Row {
    id: thirdRow

    x: 0
    y: parent.height * 2 / 3
    width: parent.width * 0.55
    height: parent.height / 3
    anchors.horizontalCenter: parent.horizontalCenter
    spacing: 18

    property double itemWidth: Math.min((width - spacing * 4) / 3, height)

    Rectangle {
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
