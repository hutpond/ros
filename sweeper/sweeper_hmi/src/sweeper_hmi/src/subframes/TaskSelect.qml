/**
*  select clean area and route
*
*/
import QtQuick 2.0
import "../basic" as Basic
import Sweeper.DataManager 1.0

Item {
  id: itemTaskSlect

  property var taskList: []
  property var currentTask: []
  property int pageIndex: 0
  property int sizePerPage: 8
  property int taskSize: 0

  property double spacing: 8
  property double itemWidth: (width - 5 * spacing) / 4.0
  property double itemHeight: (height * 0.8 - 3 * spacing) / 2.0

  signal taskSlecked(var task)

  Rectangle {
    anchors.fill: parent
    color: "#0A0A0A"
  }

  Basic.TaskSelectItem {
    id: item0

    x: spacing
    y: spacing
    width: itemWidth
    height: itemHeight

    onSelectedTask: {
      var index = pageIndex * sizePerPage
      if (index < taskSize) {
        currentTask = taskList[index]

        item1.setSelected(false)
        item2.setSelected(false)
        item3.setSelected(false)
        item4.setSelected(false)
        item5.setSelected(false)
        item6.setSelected(false)
        item7.setSelected(false)
      }
    }
  }

  Basic.TaskSelectItem {
    id: item1

    x: spacing + (spacing + itemWidth)
    y: spacing
    width: itemWidth
    height: itemHeight

    onSelectedTask: {
      var index = pageIndex * sizePerPage + 1
      if (index < taskSize) {
        currentTask = taskList[index]

        item0.setSelected(false)
        item2.setSelected(false)
        item3.setSelected(false)
        item4.setSelected(false)
        item5.setSelected(false)
        item6.setSelected(false)
        item7.setSelected(false)
      }
    }
  }


  Basic.TaskSelectItem {
    id: item2

    x: spacing + (spacing + itemWidth) * 2
    y: spacing
    width: itemWidth
    height: itemHeight

    onSelectedTask: {
      var index = pageIndex * sizePerPage + 2
      if (index < taskSize) {
        currentTask = taskList[index]

        item0.setSelected(false)
        item1.setSelected(false)
        item3.setSelected(false)
        item4.setSelected(false)
        item5.setSelected(false)
        item6.setSelected(false)
        item7.setSelected(false)
      }
    }
  }


  Basic.TaskSelectItem {
    id: item3

    x: spacing + (spacing + itemWidth) * 3
    y: spacing
    width: itemWidth
    height: itemHeight

    onSelectedTask: {
      var index = pageIndex * sizePerPage + 3
      if (index < taskSize) {
        currentTask = taskList[index]

        item0.setSelected(false)
        item1.setSelected(false)
        item2.setSelected(false)
        item4.setSelected(false)
        item5.setSelected(false)
        item6.setSelected(false)
        item7.setSelected(false)
      }
    }
  }

  Basic.TaskSelectItem {
    id: item4

    x: spacing
    y: spacing * 2 + itemHeight
    width: itemWidth
    height: itemHeight

    onSelectedTask: {
      var index = pageIndex * sizePerPage + 4
      if (index < taskSize) {
        currentTask = taskList[index]

        item0.setSelected(false)
        item1.setSelected(false)
        item2.setSelected(false)
        item3.setSelected(false)
        item5.setSelected(false)
        item6.setSelected(false)
        item7.setSelected(false)
      }
    }
  }

  Basic.TaskSelectItem {
    id: item5

    x: spacing + (spacing + itemWidth)
    y: spacing * 2 + itemHeight
    width: itemWidth
    height: itemHeight

    onSelectedTask: {
      var index = pageIndex * sizePerPage + 5
      if (index < taskSize) {
        currentTask = taskList[index]

        item0.setSelected(false)
        item1.setSelected(false)
        item2.setSelected(false)
        item3.setSelected(false)
        item4.setSelected(false)
        item6.setSelected(false)
        item7.setSelected(false)
      }
    }
  }

  Basic.TaskSelectItem {
    id: item6

    x: spacing + (spacing + itemWidth) * 2
    y: spacing * 2 + itemHeight
    width: itemWidth
    height: itemHeight

    onSelectedTask: {
      var index = pageIndex * sizePerPage + 6
      if (index < taskSize) {
        currentTask = taskList[index]

        item0.setSelected(false)
        item1.setSelected(false)
        item2.setSelected(false)
        item3.setSelected(false)
        item4.setSelected(false)
        item5.setSelected(false)
        item7.setSelected(false)
      }
    }
  }

  Basic.TaskSelectItem {
    id: item7

    x: spacing + (spacing + itemWidth) * 3
    y: spacing * 2 + itemHeight
    width: itemWidth
    height: itemHeight

    onSelectedTask: {
      var index = pageIndex * sizePerPage + 7
      if (index < taskSize) {
        currentTask = taskList[index]

        item0.setSelected(false)
        item1.setSelected(false)
        item2.setSelected(false)
        item3.setSelected(false)
        item4.setSelected(false)
        item5.setSelected(false)
        item6.setSelected(false)
      }
    }
  }

  Item {
    id: itemSelect

    anchors.top: item7.bottom
    anchors.topMargin: spacing
    anchors.bottom: parent.bottom
    anchors.bottomMargin: spacing

    anchors.verticalCenter: parent.Center
    width: parent.width * 0.15

    Image {
      anchors.fill: parent
      source: "qrc:/svg/button.svg"
    }
    Text {
      id: textButtonSelect

      text: qsTr("Select")
      font.family: "SimHei"
      font.pointSize: 18
      anchors.centerIn: parent
      color: "white"
    }

    MouseArea {
      anchors.fill: parent
      hoverEnabled: true

      onClicked: {
        DataManager.setCleaningTask(currentItem[0], currentItem[1])
        taskSlecked(currentItem)
      }
    }

  }

  Component.onCompleted: {
    taskList = DataManager.getCleanningRoute()
    taskSize = taskList.length
    var remainder = (taskList.length % 8);
    if (remainder !== 0) {
      for (var i = 0; i < 8 - remainder; ++i) {
        var task = []
        task.push("")
        task.push("")
        task.push("")
        task.push("")
        taskList.push(task)
      }
    }

    setPage(0)
  }

  function setPage(index) {
    pageIndex = index

    var taskIndex = sizePerPage * pageIndex

    item0.setTaskItem(taskList[taskIndex])
    item0.setSelected(true)

    item1.setTaskItem(taskList[taskIndex + 1])
    item1.setSelected(false)

    item2.setTaskItem(taskList[taskIndex + 2])
    item2.setSelected(false)

    item3.setTaskItem(taskList[taskIndex + 3])
    item3.setSelected(false)

    item4.setTaskItem(taskList[taskIndex + 4])
    item4.setSelected(false)

    item5.setTaskItem(taskList[taskIndex + 5])
    item5.setSelected(false)

    item6.setTaskItem(taskList[taskIndex + 6])
    item6.setSelected(false)

    item7.setTaskItem(taskList[taskIndex + 7])
    item7.setSelected(false)

    update()
  }
}
