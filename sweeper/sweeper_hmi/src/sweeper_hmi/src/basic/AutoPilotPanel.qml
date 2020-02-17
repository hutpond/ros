/**
 * 自动驾驶操作面板，包括以下部分：
 * 全局路径显示，扫地车按钮面板，
 * 自动驾驶、停止自动驾驶、靠边停车操作按钮
 *
*/
import QtQuick 2.0
import QtQuick.Layouts 1.3
import Sweeper.DataManager 1.0

Item {

  id: autoPilotPanel
  signal auto()
  signal manul()
  signal stopAuto()
  signal stopAutoBack()
  signal showErrMsg()

  property bool pause: false
  property bool errorState: false

  FullScreenView {
    id: fullView
    x: 0
    y: 0
    width: parent.width
    height: parent.height * 0.34
  }

  DashBoard {
    id: dashBoardAutoPilotPanel

    anchors.left: parent.left
    anchors.right: parent.right
    anchors.top: fullView.bottom
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
      id: textButtonAuto

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
        if (errorState) {
          console.log("pannel signal")
          autoPilotPanel.showErrMsg()
        }
        else {
          autoPilotPanel.auto()
        }
      }
    }
  }

  Item {
    id: buttonManul

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
      text: qsTr("人工驾驶")
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
        autoPilotPanel.manul()
      }
    }
  }

  Item {
    id: buttonStopBySide

    anchors.fill: buttonAuto
    visible: false

    Image {
      anchors.fill: parent
      source: "qrc:/svg/button.svg"
    }
    Text {
      id: btnText
      text: qsTr("靠边停车")
      font.family: "SimHei"
      font.pointSize: 26
      anchors.centerIn: parent
      color: "white"
    }

    MouseArea {
      anchors.fill: parent
      hoverEnabled: true

      onClicked: {
        if (pause) {
          DataManager.startAuto()
          btnText.text = qsTr("靠边停车")
        }
        else {
          DataManager.stopBySide()
          btnText.text = qsTr("恢复")
        }
        pause = !pause
      }
    }
  }

  Item {
    id: buttonStopAuto

    anchors.fill: buttonManul
    visible: false

    Image {
      anchors.fill: parent
      source: "qrc:/svg/button.svg"
    }
    Text {
      text: qsTr("停止")
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
        autoPilotPanel.stopAuto()
        fullView.setAutoMode(false)
      }
    }
  }

  Timer {
    id: timer
    interval: 200; running: false; repeat: true
    onTriggered: {
      var autoIsReady = DataManager.autoIsReady()
      if (buttonAuto.visible && autoPilot.enabled !== autoIsReady) {
        buttonAuto.enabled = autoIsReady
      }
      var autoIsQuit = DataManager.autoIsQuit()
      if (!buttonAuto.visible && autoIsQuit) {
        changeToAutoRun(false)
        autoPilotPanel.stopAutoBack()
      }

      if (autoIsQuit) {
        fullView.setAutoMode(false)
      }
    }
  }

  onVisibleChanged: {
    timer.running = visible
    pause = false
    btnText.text = qsTr("靠边停车")
  }

  function setButtonEnable(enabled) {
    buttonAuto.enabled = enabled
    buttonManul.enabled = enabled
  }

  function changeToAutoRun(flag) {
    buttonAuto.visible = !flag
    buttonManul.visible = !flag
    buttonStopBySide.visible = flag
    buttonStopAuto.visible = flag

    fullView.setAutoMode(flag)
  }

  function setPosition(lon, lat) {
    fullView.setPosition(lon, lat)
    fullView.update()
  }

  function setErrorState(state) {
    errorState = state
    textButtonAuto.text = state ? qsTr("检查错误") : qsTr("自动工作")
  }
}
