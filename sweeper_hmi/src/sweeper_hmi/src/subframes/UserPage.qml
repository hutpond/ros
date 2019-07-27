import QtQuick 2.0

Item {
  Header {
    id: header
    width: parent.width
    height: parent.height * 0.08
  }

  Rectangle {
    id: startAutoPilot
    width: parent.width * 0.26
    height: parent.height * 0.18
    x: parent.width - width - 15
    y: parent.height - height - 30
    radius: width * 0.2

    color: "cyan"

    Text {
      text: qsTr("自动工作")
      font.family: "SimHei"
      font.pointSize: 26
      anchors.centerIn: parent

      color: "white"
    }

    MouseArea {
      anchors.fill: parent
      onClicked: {

      }
    }
  }
}
