import QtQuick 2.0

Item {
  id: itemAutoPilotRunPanel
  signal stop()

  Image {
    id: mapAutoPilotrunPanel
    source: "qrc:/image/map_1.png"
    x: 0
    y: 0
    width: parent.width
    height: parent.height * 0.34
  }

  DashBoard {
    id: dashBoardAutoPilotRunPanel
    x: 0
    y: mapAutoPilotrunPanel.height
    width: parent.width
    height: parent.height * 0.26
  }

  Item {
    id: buttonAutoRun
    x: 0
    y: mapAutoPilotrunPanel.height + dashBoardAutoPilotRunPanel.height
    width: parent.width
    height: parent.height * 0.4

    Rectangle {
      width: parent.width * 0.7
      height: parent.height * 0.3
      anchors.centerIn: parent
      radius: height * 0.5
      color: "red"

      MouseArea {
        anchors.fill: parent
        hoverEnabled: true

        onEntered: {
          parent.color = "cyan"
        }
        onExited: {
          parent.color = "red"
        }
        onClicked: {
          itemAutoPilotRunPanel.stop()
        }
      }

      Text {
        text: qsTr("停止")
        font.family: "SimHei"
        font.pointSize: 26
        anchors.centerIn: parent
        color: "white"
      }
    }
  }

}
