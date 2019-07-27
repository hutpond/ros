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

  Item {
    anchors.fill: parent
    focus: true
    Keys.onEscapePressed: Qt.quit()
  }

  SubFrames.Start {
    id: start
    anchors.fill: parent
  }

  SubFrames.Check {
    id: check
    anchors.fill: parent
    visible: false
  }

  SubFrames.UserPage {
    id: userPage
    anchors.fill: parent
    visible: false
  }

  Item {
    Timer {
      interval: 2000; running: true; repeat: false
      onTriggered: {
        start.visible = false
        check.visible = true
      }
    }
  }

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
        timerShowUserPage.start()
      }
    }
  }
}
