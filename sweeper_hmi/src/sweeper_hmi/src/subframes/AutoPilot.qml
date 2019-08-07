import QtQuick 2.0
import "../basic" as Basic

Item {

  id: itemAutoPilot
  property double space: 10
  signal auto()
  signal manul()

  Basic.Header {
    id: headerAutoPilot
    title: "AI智能扫地机"
    width: parent.width
    height: parent.height * 0.08
  }

  Basic.CarLocationState {
    id: locationState
    x: space
    y: headerAutoPilot.height + space
    width: parent.width * 0.6
    height: parent.height - headerAutoPilot.height - space * 2;
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
      }
      else {
        waitStartAuto.setText(index)
        -- index
      }
    }
  }

  Basic.AutoPilotPanel {
    id: autoPilotPanel
    x: locationState.width + 2 * space
    y: headerAutoPilot.height + space
    width: parent.width - locationState.width - 3 * space
    height: parent.height - headerAutoPilot.height - 2 * space;

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
    onVisibleChanged: {
      if (visible) {
        setButtonEnable(true)
      }
    }
  }
}
