import QtQuick 2.0
import "../basic" as Basic

Item {
  property double space: 10

  signal cancel()

  Rectangle {
    id: backgroud
    anchors.fill: parent
    color: "#0A0A0A"
  }

  Basic.TaskDispaly {
    id: taskDisplay

    anchors.left: parent.left
    anchors.leftMargin: space
    anchors.top: parent.top
    anchors.topMargin: 35
    anchors.bottom: parent.bottom

    width: parent.width * 0.65
  }

  Basic.TaskReportPannel {
    id: pannel

    anchors.left: taskDisplay.right
    anchors.leftMargin: space
    anchors.right: parent.right
    anchors.rightMargin: space * 2

    anchors.top: parent.top
    anchors.topMargin: 35
    anchors.bottom: parent.bottom

    onSend: {

    }

    onCancel: {
      parent.cancel()
    }
  }
}
