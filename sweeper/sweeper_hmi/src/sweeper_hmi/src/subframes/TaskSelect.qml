import QtQuick 2.0
import "../basic" as Basic

Item {
  id: itemTaskSlect

//  Basic.Header {
//    title: "工作任务选择"
//    width: parent.width
//    height: parent.height * 0.08
//  }

  signal taskSlecked(int index)

  MouseArea {
    anchors.fill: parent
    onClicked: {
      itemTaskSlect.taskSlecked(0)
    }
  }
}
