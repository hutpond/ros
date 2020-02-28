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
  property Item currentItem: start
  property bool errorState: false

  color: "#ffffff"
  title: qsTr("Conway’s Game of Life")
  //visibility: Window.FullScreen
  //flags: Qt.FramelessWindowHint

  // quit
//  Item {
//    anchors.fill: parent
//    focus: true
//    Keys.onEscapePressed: Qt.quit()
//  }

  // header
  SubFrames.Header {
    id: header

    anchors.left: parent.left
    anchors.right: parent.right
    anchors.top: parent.top
    height: parent.height * 87 / 768.0

    function onShowErrMessage() {
      vehicleErrMsg.visible = !vehicleErrMsg.visible
      currentItem.visible = !vehicleErrMsg.visible
    }

    Component.onCompleted: {
      showErrMessage.connect(onShowErrMessage)
    }
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
        DataManager.startCheck()
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

  // vehicle error message
  SubFrames.VehicleErrMessage {
    id: vehicleErrMsg
    anchors.left: parent.left
    anchors.right: parent.right
    anchors.top: header.bottom
    anchors.bottom: parent.bottom
    visible: false
  }

  // user page
  SubFrames.UserPage {
    id: userPage
    anchors.left: parent.left
    anchors.right: parent.right
    anchors.top: header.bottom
    anchors.bottom: parent.bottom
    visible: false

    onStartAutoPilot: {
      if (taskSelect.taskList.length == 0 || taskSelect.taskList[0].length === 0 ||
          taskSelect.taskList[0][0] === "") {
        visible = false
        currentItem = taskSelect
        currentItem.visible = true
      }
      else {
        userPage.visible = false
        autoPilot.visible = true
      }
    }

    onVisibleChanged: {
      header.visible = true
      if (visible) {
        header.title = "AI智能扫地机"
        currentItem = this
      }
    }

    onShowErrMsg: {
      vehicleErrMsg.visible = true
      currentItem.visible = false
    }

    onSelectTask: {
      visible = false
      currentItem = taskSelect
      currentItem.visible = true
    }

  }

  // task page
  SubFrames.TaskSelect {
    id: taskSelect
    anchors.left: parent.left
    anchors.right: parent.right
    anchors.top: header.bottom
    anchors.bottom: parent.bottom
    visible: false

    onTaskSlecked: {
      taskSelect.visible = false
      currentItem = userPage
      userPage.visible = true
      if (task.length > 0) {
        userPage.setSelectedTask(task)
      }
    }
  }

  // auto pilot start
  SubFrames.AutoPilot {
    id: autoPilot
    anchors.left: parent.left
    anchors.right: parent.right
    anchors.top: header.bottom
    anchors.bottom: parent.bottom
    visible: false

    onAuto: {
      DataManager.startAuto()
    }
    onStopAuto: {
      DataManager.stopAuto()
      autoPilot.visible = false
      //userPage.visible = true
      taskReport.visible = true
    }
    onStopAutoBack: {
      DataManager.stopAuto()
      autoPilot.visible = false
      currentItem = userPage

      //vehicleErrMsg.visible = true
      currentItem.visible = true
    }

    onManul: {
      autoPilot.visible = false
      userPage.visible = true
    }
    onVisibleChanged: {
      header.visible = true;
      if (visible) {
        header.title = "AI智能扫地机"
        currentItem = this
      }
    }

    onShowErrMsg: {
      console.log("subframe signal")
      vehicleErrMsg.visible = true
      currentItem.visible = false
    }
  }

  // task report
  SubFrames.TaskReport {
    id: taskReport
    anchors.left: parent.left
    anchors.right: parent.right
    anchors.top: header.bottom
    anchors.bottom: parent.bottom
    visible: false

    onCancel: {
      autoPilot.visible = true
      visible = false
    }

    onVisibleChanged: {
      header.visible = true;
      if (visible) {
        header.title = "任务报告"
        currentItem = this
      }
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
  Connections {
    target: DataManager

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
      timerError.running = true
    }
  }

  // check error
  Timer {
    id: timerError
    interval: 200; running: false; repeat: true
    onTriggered: {
      var curErrState = (DataManager.infos.length > 0)
      if (errorState !== curErrState) {
        errorState = curErrState
        header.setErrState(errorState)
        userPage.setErrState(errorState)
        autoPilot.setErrState(errorState)

        vehicleErrMsg.visible = (errorState && !header.visible)
        currentItem.visible = !errorState
      }
    }
  }

}
