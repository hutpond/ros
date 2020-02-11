import QtQuick 2.0
import "../basic" as Basic
import Sweeper.DataManager 1.0

Item {
  id: itemUserPage

  property double leftWidth: 0.65
  property double xSpace: 10
  property double ySpace: 10

  signal startAutoPilot()

  Rectangle {
    anchors.fill: parent
    color: "#0A0A0A"
  }

  Basic.UserPageState {
    id: userPageState

    anchors.left: parent.left
    anchors.leftMargin: 30
    anchors.top: parent.top
    anchors.topMargin: 35

    width: parent.width * 637.0 / 1024
    height: parent.height * 480.0 / 768
  }

  Basic.UserPageBoard {
    id: userPageBoard

    anchors.left: userPageState.right
    anchors.leftMargin: 30
    anchors.top: userPageState.top
    anchors.bottom: userPageState.bottom
    anchors.right: parent.right
    anchors.rightMargin: 30
  }

  Basic.UserPageDisplayBoard {
    id: displayBoard

    anchors.left: userPageState.left
    anchors.right: userPageState.right
    anchors.top: userPageState.bottom
    anchors.topMargin: 40
    anchors.bottom: parent.bottom
    anchors.bottomMargin: 55
  }

  Item {
    id: startAutoPilot

    anchors.top: displayBoard.top
    anchors.bottom: displayBoard.bottom
    anchors.left: userPageBoard.left
    anchors.right: userPageBoard.right

    Image {
      anchors.fill: parent
      source: "qrc:/svg/button.svg"
    }

    Text {
      text: qsTr("自动工作")
      font.family: "SimHei"
      font.pointSize: 26
      anchors.centerIn: parent

      color: "white"
    }

    MouseArea {
      anchors.fill: parent
      onClicked: {
        itemUserPage.startAutoPilot()
      }
    }
  }

  /**
   * 定时器，读电池电量
  */
  Timer {
    id: timerVehicleUserPage
    interval: 200; running: false; repeat: true
    onTriggered: {
      // velocity
      var valueVelocity = DataManager.getProperty("data_velocity")
      if("number" == typeof valueVelocity){
        valueVelocity *= 3.6
        valueVelocity = valueVelocity.toFixed(1)
        userPageState.setVelocity(valueVelocity)
      }
      // battery level
      var valueBattery = DataManager.getProperty("data_battery_remaining_capacity")
      if("number" == typeof valueBattery){
        valueBattery = Math.round(valueBattery)
        displayBoard.setBatteryLevel(valueBattery)
      }
      // gear mode
      var valueGearMode = DataManager.getProperty("data_gear")
      if("number" == typeof valueBattery){
        userPageState.setVehicleGear(valueGearMode)
      }
    }
  }

  onVisibleChanged: {
    timerVehicleUserPage.running = visible
  }

}
