import QtQuick 2.0
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3
import "../basic" as Basic
import Sweeper.DataManager 1.0

Item {
  id: check
  property bool checkRet: true
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

  function startTimer() {
    checkRet = true
    checkVehicle.setState(Basic.CheckStateBox.None)
    checkBoxSys.setState(Basic.CheckStateBox.None)
    checkBoxSensor.setState(Basic.CheckStateBox.None)
    checkBoxEnv.setState(Basic.CheckStateBox.None)
    canvasCheck.startTimer()
  }
  function stopTimer() {
    canvasCheck.stopTimer()
  }
  function setCheckedResult(type, ret) {
    if (checkRet && !ret) {
      checkRet = ret
    }
    if (type === DataManager.CheckVehicle) {
      checkVehicle.setState(ret ? Basic.CheckStateBox.Checked : Basic.CheckStateBox.UnChecked)
    }
    else if (type === DataManager.CheckSystem){
      checkBoxSys.setState(ret ? Basic.CheckStateBox.Checked : Basic.CheckStateBox.UnChecked)
    }
    else if (type === DataManager.CheckSensor){
      checkBoxSensor.setState(ret ? Basic.CheckStateBox.Checked : Basic.CheckStateBox.UnChecked)
    }
    else if (type === DataManager.CheckAlgorithm){
      checkBoxEnv.setState(ret ? Basic.CheckStateBox.Checked : Basic.CheckStateBox.UnChecked)
    }
  }
  function setCheckedStep(step) {
    canvasCheck.setPercent(step)
  }
}
