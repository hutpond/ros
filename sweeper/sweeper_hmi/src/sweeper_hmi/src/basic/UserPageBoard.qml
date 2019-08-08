import QtQuick 2.0

Item {

  property double ySpace: 20

  PerceptionData {
    id: perceptionData

    anchors.top: parent.top
    anchors.left: parent.left
    anchors.right: parent.right

    height: (parent.height - ySpace) / 2.0
  }

  Camera {
    id: camera

    anchors.top: perceptionData.bottom
    anchors.topMargin: ySpace
    anchors.bottom: parent.bottom
    anchors.left: parent.left
    anchors.right: parent.right
  }
}
