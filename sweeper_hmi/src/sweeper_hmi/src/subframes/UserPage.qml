import QtQuick 2.0
import "../basic" as Basic

Item {
  id: itemUserPage

  property double leftWidth: 0.65
  property double xSpace: 10
  property double ySpace: 10

  signal startAutoPilot()

  Basic.Header {
    id: headerUserPage
    title: "AI智能扫地机"
    width: parent.width
    height: parent.height * 0.08
  }

  Basic.UserPageState {
    id: userPageState

    anchors.left: parent.left
    anchors.leftMargin: xSpace
    anchors.top: headerUserPage.bottom
    anchors.topMargin: ySpace
    anchors.bottom: parent.bottom
    anchors.bottomMargin: ySpace

    width: parent.width * 0.6
  }

  Basic.UserPageBoard {
    id: userPageBoard

    anchors.left: userPageState.right
    anchors.leftMargin: xSpace
    anchors.top: userPageState.top
    anchors.bottom: userPageState.bottom
    anchors.right: parent.right
    anchors.rightMargin: xSpace
  }
}
