import QtQuick 2.0
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3
import Sweeper 1.0
import "../basic" as Basic

Item {
  id: check
  property int pointSize: 24

  Rectangle {
    anchors.fill: parent
    color: "#000001"
  }

  Image {
    anchors.fill: parent
    source: "qrc:/image/check.png"
  }

  Image {
    source: "qrc:/image/deepblue.png"
    x: 37
    y: 30
    width: 135
    height: 132
  }

  Text {
    text: "系统自检中..."
    font.family: "SimHei"
    font.pointSize: 30
    color: "white"

    y: 140
    anchors.horizontalCenter: parent.horizontalCenter
  }

  Item {
    y: parent.height * 0.8
    x: parent.width * 0.18
    width: parent.width * 0.7
    height: parent.height * 0.08

    Basic.CheckStateBox {
      id: checkVehicle
      text: "车辆"

      width: parent.width * 0.22
      height: parent.height

      anchors.left: parent.left
      anchors.leftMargin: 20
      anchors.verticalCenter: parent.verticalCenter
    }

    Basic.CheckStateBox {
      id: checkBoxSys
      text: "系统"

      width: parent.width * 0.22
      height: parent.height

      anchors.left: checkVehicle.right
      anchors.leftMargin: 20
      anchors.verticalCenter: parent.verticalCenter
    }

    Basic.CheckStateBox {
      id: checkBoxSensor
      text: "传感器"

      width: parent.width * 0.22
      height: parent.height

      anchors.left: checkBoxSys.right
      anchors.leftMargin: 20
      anchors.verticalCenter: parent.verticalCenter
    }

    Basic.CheckStateBox {
      id: checkBoxEnv
      text: "算法"

      width: parent.width * 0.22
      height: parent.height

      anchors.left: checkBoxSensor.right
      anchors.leftMargin: 20
      anchors.verticalCenter: parent.verticalCenter
    }
  }

  Basic.CheckCanvas {
    id: canvasCheck

    width: parent.width
    height: parent.height * 0.371
    anchors.centerIn: parent
  }

  SelfChecking {
    onStepChanged: {
      if (step === SelfChecking.StepSysSucceed) {
        checkBoxSys.setState(Basic.CheckStateBox.Checked)
      }
      else if (step === SelfChecking.StepSensorFailed) {
        checkBoxSensor.setState(Basic.CheckStateBox.UnChecked)
      }
      else if (step === SelfChecking.StepEnvSucceed) {
        checkBoxEnv.setState(Basic.CheckStateBox.UnChecked)
      }
    }
  }

  Timer {
    id: timerRead
    property int index: 0

    interval: 1000; running: false; repeat: true
    onTriggered: {
      ++ index
      if (index === 2) {
        check.checked(true)
      }
    }
  }

  signal checked(bool flag)

  function startTimer() {
    timerRead.index = 0
    timerRead.start()
    canvasCheck.startTimer()
  }
  function stopTimer() {
    timerRead.stop()
    canvasCheck.stopTimer()
  }
}
