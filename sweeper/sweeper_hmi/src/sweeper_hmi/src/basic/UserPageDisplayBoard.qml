import QtQuick 2.0

Item {

  Image {
    id: battery

    anchors.left: parent.left
    anchors.top: parent.top

    width: 27
    height: 21

    source: "qrc:/svg/battery.svg"
  }

  Rectangle {
    id: batteryUsed
    color: "#081A2E"

    property int used: 48

    anchors.top: battery.top
    anchors.bottom: battery.bottom
    anchors.left: battery.right
    anchors.leftMargin: 20
    anchors.right: parent.right

    Rectangle {
      id: rectUsed
      color: "#0989FF"

      anchors.top: parent.top
      anchors.bottom: parent.bottom
      anchors.left: parent.left
      width: parent.width * batteryUsed.used / 100
    }

    Text {
      id: name

      anchors.top: parent.top
      anchors.bottom: parent.bottom
      anchors.right: rectUsed.right
      anchors.rightMargin: 10

      font.family: "SimHei"
      font.pointSize: 12
      verticalAlignment: Text.AlignVCenter
      color: "white"

      text: batteryUsed.used + "%"
    }
  }

  Item {
    id: board

    anchors.top: battery.bottom
    anchors.topMargin: 30
    anchors.bottom: parent.bottom
    anchors.left: battery.left
    anchors.right: batteryUsed.right

    property double space: (width - height * 9) / 8.0

    Row {
      spacing: board.space

      Image {
        width: board.height
        height: width
        source: "qrc:/svg/brake.svg"
      }
      Image {
        width: board.height
        height: width
        source: "qrc:/svg/positionLamp.svg"
      }
      Image {
        width: board.height
        height: width
        source: "qrc:/svg/highBeam.svg"
      }
      Image {
        width: board.height
        height: width
        source: "qrc:/svg/lowBeam.svg"
      }
      Image {
        width: board.height
        height: width
        source: "qrc:/svg/spout.svg"
      }
      Image {
        width: board.height
        height: width
        source: "qrc:/svg/pallet.svg"
      }
      Image {
        width: board.height
        height: width
        source: "qrc:/svg/leftSignalLight.svg"
      }
      Image {
        width: board.height
        height: width
        source: "qrc:/svg/rightSignalLight.svg"
      }
      Image {
        width: board.height
        height: width
        source: "qrc:/svg/setting.svg"
      }
    }

  }

}
