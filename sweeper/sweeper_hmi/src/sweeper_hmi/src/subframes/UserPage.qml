import QtQuick 2.0
import "../basic" as Basic

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

  Basic.Header {
    id: headerUserPage
    title: "AI智能扫地机"
    width: parent.width
    height: parent.height * 87.0 / 768
  }

  Basic.UserPageState {
    id: userPageState

    anchors.left: parent.left
    anchors.leftMargin: 30
    anchors.top: headerUserPage.bottom
    anchors.topMargin: 35

    width: parent.width * 637.0 / 1024
    height: parent.height * 449.0 / 768
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

  Item {
    id: startAutoPilot

    //anchors.top: gearsUserPage.bottom
    //anchors.topMargin: ySpace
    anchors.bottom: parent.bottom
    anchors.bottomMargin: ySpace
    width: parent.width

    Rectangle {
      id: pilotType

    }

    Rectangle {
      height: parent.height * 0.1
      //radius: width * 0.2

      //color: "cyan"

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
          //itemUserPage.startAutoPilot()
        }
      }
    }
  }

}
