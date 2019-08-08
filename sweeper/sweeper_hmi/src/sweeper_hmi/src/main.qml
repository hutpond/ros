import QtQuick 2.8
import QtQuick.Window 2.2
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3
import "subframes" as SubFrames
import Sweeper 1.0

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

    onChecked: {
      check.stopTimer()
      check.visible = false
      if (!flag) {
        checkErrMsg.visible = true
      }
      else {
        userPage.visible = true
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
      taskSelect.visible = true
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
      autoPilot.visible = false
      autoPilotRun.visible = true
    }

    onManul: {
      userPage.visible = true
      autoPilot.visible = false
    }
  }

  // auto pilot run
  SubFrames.AutoPilotRun {
    id: autoPilotRun
    anchors.fill: parent
    visible: false

    onStop: {
      autoPilotRun.visible = false
      autoPilot.visible = true
    }
  }

  // stop on start page for second
  Item {
    Timer {
      interval: 5000; running: true; repeat: false
      onTriggered: {
        start.visible = false
        check.visible = true
        check.startTimer()
      }
    }
  }

  // check function
  SelfChecking {
    Timer {
      id: timerShowUserPage
      interval: 800; running: false; repeat: false
      onTriggered: {
        check.visible = false
        userPage.visible = true
      }
    }
    onStepChanged: {
      if (step === SelfChecking.StepEnvSucceed) {
//        timerShowUserPage.start()
      }
    }
  }
}
