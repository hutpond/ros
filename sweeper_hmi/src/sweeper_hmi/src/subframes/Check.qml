import QtQuick 2.0
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3
import Sweeper 1.0

Item {
  property int pointSize: 30

  Image {
    x: parent.width * 0.02
    y: x / 2
    width: parent.width * 0.05
    height: width
    source: "qrc:/image/deepblue.png"
  }

  Text {
    text: "系统自检"
    font.family: "SimHei"
    font.pointSize: pointSize
    color: "black"

    y: parent.height * 0.15
    anchors.horizontalCenter: parent.horizontalCenter
  }

  RowLayout {
    y: parent.height * 0.8
    x: parent.width * 0.18
    width: parent.width * 0.7
    Text {
      text: "车辆"
      font.family: "SimHei"
      font.pointSize: pointSize
      color: "black"
    }
    CheckBox {
      id: checkBoxSys
      text: "系统"
      font.family: "SimHei"
      font.pointSize: pointSize
      focusPolicy: Qt.NoFocus

      MouseArea {
        anchors.fill: parent
      }
    }
    CheckBox {
      id: checkBoxSensor
      text: "传感器"
      font.family: "SimHei"
      font.pointSize: pointSize
      focusPolicy: Qt.NoFocus

      MouseArea {
        anchors.fill: parent
      }
    }
    CheckBox {
      id: checkBoxEnv
      text: "算法"
      font.family: "SimHei"
      font.pointSize: pointSize
      focusPolicy: Qt.NoFocus

      MouseArea {
        anchors.fill: parent
      }
    }
  }

  SelfChecking {
    onStepChanged: {
      if (step === SelfChecking.StepSysSucceed) {
        checkBoxSys.checked = true
      }
      else if (step === SelfChecking.StepSensorFailed) {
        checkBoxSensor.checked = false
      }
      else if (step === SelfChecking.StepEnvSucceed) {
        checkBoxEnv.checked = true
      }
    }
  }
}
