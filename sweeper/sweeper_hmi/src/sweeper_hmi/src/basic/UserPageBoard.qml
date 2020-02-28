/**
* display the selected route
*
*/
import QtQuick 2.0

Item {

  property double ySpace: 20

  signal selectTask()

  Item {
    id: cleaningArea

    anchors.top: parent.top
    anchors.left: parent.left
    anchors.right: parent.right

    height: (parent.height - ySpace) / 2.0

    Rectangle {
      anchors.fill: parent
      color: "lightGray"
    }

    Image {
      id: imageArea
      anchors.fill: parent
      source: ""
    }
  }

  Item {
    id: cleaningRoute

    anchors.top: cleaningArea.bottom
    anchors.topMargin: ySpace
    anchors.bottom: parent.bottom
    anchors.left: parent.left
    anchors.right: parent.right

    Rectangle {
      anchors.fill: parent
      color: "lightGray"
    }

    Image {
      id: imageRoute
      anchors.fill: parent
      source: ""
    }
  }

  MouseArea {
    anchors.fill: parent
    hoverEnabled: true

    onEntered: {
    }
    onExited: {
    }
    onClicked: {
      selectTask()
    }
  }

  function setSelectedTask(task) {
    if (task.length === 4) {
      imageArea.source = "file://" + task[2]
    }
  }
}
