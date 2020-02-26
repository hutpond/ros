import QtQuick 2.0

Item {

  property bool selected: false
  property var taskItem: []

  signal selectedTask(var task)

  Rectangle {
    id: rectBkg
    anchors.fill: parent
    color: "gray"
    visible: true
  }

  Image {
    id: routeImage
    source: ""

    x: 0
    y: 0
    width: parent.width
    height: parent.height * 0.7
  }

  Item {
    id: itemSite

    x: 0
    y: parent.height * 0.7
    width: parent.width
    height: parent.height * 0.15

    Text {
      id: siteName
      text: qsTr("")
      color: "black"

      anchors.centerIn: parent
      font.family: "SimHei"
      font.pointSize: 14
    }
  }

  Item {
    id: itemRoute

    x: 0
    y: parent.height * 0.85
    width: parent.width
    height: parent.height * 0.15

    Text {
      id: routeName
      text: qsTr("")
      color: "black"

      anchors.centerIn: parent
      font.family: "SimHei"
      font.pointSize: 14
    }
  }

  MouseArea {
    anchors.fill: parent
    hoverEnabled: true

    onClicked: {
      if (taskItem.length > 0) {
        selectedTask(taskItem)
        setSelected(true)
      }
    }
  }

  function setTaskItem(task) {
    if (tast.length >= 3) {
      taskItem = task
      siteName.text = task[0]
      routeName.text = task[1]
      routeImage.source = task[2]
    }
  }

  function setSelected(value) {
    if (selected !== value) {
      selected = value
      if (value) {
        rectBkg.color = "lightBlue"
      }
      else {
        rectBkg.color = "gray"
      }
      update()
    }
  }
}
