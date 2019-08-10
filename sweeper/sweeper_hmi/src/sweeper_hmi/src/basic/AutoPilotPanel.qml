import QtQuick 2.0
import QtQuick.Layouts 1.3

Item {

  id: autoPilotPanel
  signal auto()
  signal manul()
  signal stopAuto()

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

    anchors.left: parent.left
    anchors.right: parent.right
    anchors.top: mapAutoPilotPanel.bottom
    anchors.topMargin: 30
    height: parent.height * 0.15
  }

  Item {
    id: buttonAuto

    anchors.left: dashBoardAutoPilotPanel.left
    anchors.right: dashBoardAutoPilotPanel.right
    anchors.top: dashBoardAutoPilotPanel.bottom
    anchors.topMargin: 40
    height: parent.height * 0.15

    Image {
      anchors.fill: parent
      source: "qrc:/svg/button.svg"
    }
    Text {
      text: qsTr("启动")
      font.family: "SimHei"
      font.pointSize: 26
      anchors.centerIn: parent
      color: "white"
    }

    MouseArea {
      anchors.fill: parent
      hoverEnabled: true

      onEntered: {
      }
      onExited: {
      }
      onClicked: {
        autoPilotPanel.auto()
      }
    }
  }

  Item {
    id: buttonManul
    property bool manulRun: true

    anchors.left: buttonAuto.left
    anchors.right: buttonAuto.right
    anchors.bottom: parent.bottom
    anchors.bottomMargin: 20
    height: buttonAuto.height

    Image {
      anchors.fill: parent
      source: "qrc:/svg/button.svg"
    }
    Text {
      text: buttonManul.manulRun ? qsTr("人工驾驶") : qsTr("停止")
      font.family: "SimHei"
      font.pointSize: 26
      anchors.centerIn: parent
      color: "white"
    }

    MouseArea {
      anchors.fill: parent
      hoverEnabled: true

      onEntered: {
      }
      onExited: {
      }
      onClicked: {
        if (buttonManul.manulRun)
          autoPilotPanel.manul()
        else
        {
          autoPilotPanel.stopAuto()
        }
      }
    }
  }

  function setButtonEnable(enabled) {
    if (enabled) {
      buttonAuto.visible =true
      buttonManul.manulRun = true
    }
    buttonAuto.enabled = enabled
  }

  function changeToAutoRun() {
    buttonAuto.visible =false
    buttonManul.manulRun = false
  }
}
