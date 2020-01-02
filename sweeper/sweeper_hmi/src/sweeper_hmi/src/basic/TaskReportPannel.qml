/**
  * 任务上报操作面板，发送报告、返回
  */
import QtQuick 2.0

Item {
  id: taskPannel

  signal send()
  signal cancel()

  Image {
    id: mapTask
    source: "qrc:/image/map_1.png"

    x: 0
    y: 0
    width: parent.width
    height: parent.height * 0.34
  }

  DashBoard {
    id: dashBoardTask

    anchors.left: parent.left
    anchors.right: parent.right
    anchors.top: mapTask.bottom
    anchors.topMargin: 30
    height: parent.height * 0.15
  }

  Item {
    id: buttonSendReport

    anchors.left: dashBoardTask.left
    anchors.right: dashBoardTask.right
    anchors.top: dashBoardTask.bottom
    anchors.topMargin: 40
    height: parent.height * 0.15

    Image {
      anchors.fill: parent
      source: "qrc:/svg/button.svg"
    }
    Text {
      text: qsTr("发送报告")
      font.family: "SimHei"
      font.pointSize: 26
      anchors.centerIn: parent
      color: "white"
    }

    MouseArea {
      anchors.fill: parent
      hoverEnabled: true

      onEntered: {
      }
      onExited: {
      }
      onClicked: {
        taskPannel.send()
      }
    }
  }

  Item {
    id: buttonCancel

    anchors.left: buttonSendReport.left
    anchors.right: buttonSendReport.right
    anchors.bottom: parent.bottom
    anchors.bottomMargin: 20
    height: buttonSendReport.height

    Image {
      anchors.fill: parent
      source: "qrc:/svg/button.svg"
    }
    Text {
      text: qsTr("返回")
      font.family: "SimHei"
      font.pointSize: 26
      anchors.centerIn: parent
      color: "white"
    }

    MouseArea {
      anchors.fill: parent
      hoverEnabled: true

      onEntered: {
      }
      onExited: {
      }
      onClicked: {
        taskPannel.cancel()
      }
    }
  }
}
