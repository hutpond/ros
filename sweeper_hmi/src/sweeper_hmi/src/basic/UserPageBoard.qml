import QtQuick 2.0

Item {

  property double ySpace: 10

  PerceptionData {
    id: perceptionData

    x: 0
    y: 0
    width: parent.width
    height: parent.height * 0.3
  }

  Camera {
    id: camera

    x: 0
    width: parent.width
    anchors.top: perceptionData.bottom
    anchors.topMargin: ySpace
    height: parent.height * 0.3
  }

  Gears {
    id: gearsUserPage

    anchors.horizontalCenter: camera.horizontalCenter
    width: parent.width * 0.7

    anchors.top: camera.bottom
    anchors.topMargin: ySpace
    height: parent.height * 0.1
  }

  Item {
    id: startAutoPilot

    anchors.top: gearsUserPage.bottom
    anchors.topMargin: ySpace
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
