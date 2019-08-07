import QtQuick 2.0
import QtQuick.Layouts 1.3

Item {

  id: autoPilotPanel
  signal auto()
  signal manul()

  Image {
    id: mapAutoPilotPanel
    source: "qrc:/image/map_1.png"
    x: 0
    y: 0
    width: parent.width
    height: parent.height * 0.34
  }

  DashBoard {
    id: dashBoardAutoPilotPanel
    x: 0
    y: mapAutoPilotPanel.height
    width: parent.width
    height: parent.height * 0.26
  }

  Item {
    id: buttonAuto
    x: 0
    y: mapAutoPilotPanel.height + dashBoardAutoPilotPanel.height
    width: parent.width
    height: parent.height * 0.2

    Rectangle {
      width: parent.width * 0.7
      height: parent.height * 0.7
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
          autoPilotPanel.auto()
        }
      }

      Text {
        text: qsTr("启动")
        font.family: "SimHei"
        font.pointSize: 26
        anchors.centerIn: parent
        color: "white"
      }
    }
  }

  Item {
    id: buttonManul
    x: 0
    y: mapAutoPilotPanel.height + dashBoardAutoPilotPanel.height + buttonAuto.height
    width: parent.width
    height: parent.height * 0.2

    Rectangle {
      width: parent.width * 0.7
      height: parent.height * 0.7
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
          autoPilotPanel.manul()
        }
      }

      Text {
        text: qsTr("人工驾驶")
        font.family: "SimHei"
        font.pointSize: 26
        anchors.centerIn: parent
        color: "white"
      }
    }
  }

  function setButtonEnable(enabled) {
    buttonAuto.enabled = enabled
    buttonManul.enabled = enabled
  }
}
