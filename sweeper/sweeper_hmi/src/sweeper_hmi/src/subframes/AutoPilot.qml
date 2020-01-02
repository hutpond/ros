import QtQuick 2.0
import "../basic" as Basic

Item {

  id: itemAutoPilot
  property double space: 10
  signal auto()
  signal stopAuto()
  signal stopAutoBack()
  signal manul()

  Rectangle {
    id: backgroud
    anchors.fill: parent
    color: "#0A0A0A"
  }

  Basic.CarLocationState {
    id: locationState

    anchors.left: parent.left
    anchors.leftMargin: space
    anchors.top: parent.top
    anchors.topMargin: 35
    anchors.bottom: parent.bottom

    width: parent.width * 0.65
  }

  Basic.AutoPilotPanel {
    id: autoPilotPanel

    anchors.left: locationState.right
    anchors.leftMargin: space
    anchors.right: parent.right
    anchors.rightMargin: space * 2

    anchors.top: parent.top
    anchors.topMargin: 35
    anchors.bottom: parent.bottom

    onAuto: {
      timerStartAuto.index = 3
      waitStartAuto.setText(timerStartAuto.index)
      waitStartAuto.visible = true
      setButtonEnable(false)
      timerStartAuto.start()
    }
    onManul: {
      itemAutoPilot.manul()
    }
    onStopAuto: {
      itemAutoPilot.stopAuto()
    }
    onStopAutoBack: {
      itemAutoPilot.stopAutoBack()
    }

    onVisibleChanged: {
      if (visible) {
        setButtonEnable(true)
      }
    }
  }

  Rectangle {
    id: waitStartAuto

    width: parent.width * 0.3
    height: width
    radius: width / 2
    anchors.centerIn: parent
    z: 1

    color: "white"
    opacity: 1
    border.color: "gray"
    border.width: 2
    visible: false
    Text {
      id: waitStartAutoText
      anchors.centerIn: parent

      font.family: "SimHei"
      font.pointSize: 42
      color: "black"

      text: qsTr("3")
    }

    function setText(index) {
      waitStartAutoText.text = index
    }
  }

  Timer {
    id: timerStartAuto
    property var index: 30
    interval: 1000; running: false; repeat: true
    onTriggered: {
      if (index == -1) {
        timerStartAuto.stop()
        waitStartAuto.visible = false
        itemAutoPilot.auto()
        autoPilotPanel.changeToAutoRun(true)
      }
      else {
        waitStartAuto.setText(index)
        -- index
      }
    }
  }

  onVisibleChanged: {
    if (visible) {
      autoPilotPanel.changeToAutoRun(false)
    }
  }
}
