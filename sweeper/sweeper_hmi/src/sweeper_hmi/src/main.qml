import QtQuick 2.8
import QtQuick.Window 2.2
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3
import "subframes" as SubFrames
import Sweeper.DataManager 1.0

Window {
  id: root
  visible: true
  width: 1024
  height: 768
  minimumWidth: 475
  minimumHeight: 300

  color: "#ffffff"
  title: qsTr("Conway’s Game of Life")
  //visibility: Window.FullScreen
  flags: Qt.FramelessWindowHint

  // quit
  Item {
    anchors.fill: parent
    focus: true
    Keys.onEscapePressed: Qt.quit()
  }

  // start page
  SubFrames.Start {
    id: start
    anchors.fill: parent
  }

  // check page
  SubFrames.Check {
    id: check
    anchors.fill: parent
    visible: false

    onVisibleChanged: {
      if (visible) {
        dataManager.startCheck()
        check.startTimer()
      }
      else {
        check.stopTimer()
      }
    }
  }

  // check error message
  SubFrames.CheckErrMessage {
    id: checkErrMsg
    anchors.fill: parent
    visible: false

    onReCheck: {
      checkErrMsg.visible = false
      check.visible = true
      check.startTimer()
    }
  }

  // user page
  SubFrames.UserPage {
    id: userPage
    anchors.fill: parent
    visible: false

    onStartAutoPilot: {
      userPage.visible = false
      autoPilot.visible = true
    }
  }

  // task page
  SubFrames.TaskSelect {
    id: taskSelect
    anchors.fill: parent
    visible: false

    onTaskSlecked: {
      taskSelect.visible = false
      autoPilot.visible = true
    }
  }

  // auto pilot start
  SubFrames.AutoPilot {
    id: autoPilot
    anchors.fill: parent
    visible: false

    onAuto: {
      dataManager.startAuto()
    }
    onStopAuto: {
      dataManager.stopAuto()
      userPage.visible = true
      autoPilot.visible = false
    }
    onManul: {
      userPage.visible = true
      autoPilot.visible = false
    }
  }

  // stop on start page for second
  Item {
    Timer {
      interval: 5000; running: true; repeat: false
      onTriggered: {
        start.visible = false
        check.visible = true
      }
    }
  }

  // check function
  DataManager {
    id: dataManager

    onCheckEnd: {
      check.setCheckedResult(type, ret)
    }

    onStepChanged: {
      check.setCheckedStep(step)
    }

    onStopCheck: {
      check.visible = false
      if (check.checkRet) {
        userPage.visible = true
      }
      else {
        checkErrMsg.visible = true
      }
    }
  }
}
