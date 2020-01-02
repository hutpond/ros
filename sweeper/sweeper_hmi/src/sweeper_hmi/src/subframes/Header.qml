import QtQuick 2.0
import Sweeper.DataManager 1.0

Item {

  id: itemHeader
  property string title
  signal showErrMessage()

  Rectangle {
    anchors.fill: parent
    color: "#0A0A0A"
  }

  Image {
    id: backgroud
    anchors.fill: parent
    source: "qrc:/svg/headerBack.svg"
  }

  Image {
    id: img4G
    source: "qrc:/svg/4G.svg"

    x: 25
    height: 22
    width: 22
    anchors.verticalCenter: parent.verticalCenter
  }

  Text {
    id: text4G
    text: qsTr("4G")
    font.family: "SimHei"
    font.pointSize: 14
    color: "white"

    anchors.left: img4G.right
    anchors.leftMargin: 10
    anchors.verticalCenter: parent.verticalCenter
  }

  Image {
    id: imgGps
    source: "qrc:/image/GPS.png"

    height: 22
    width: 30

    anchors.left: text4G.right
    anchors.leftMargin: 10
    anchors.verticalCenter: parent.verticalCenter
  }

  Image {
    id: imgWarning
    source: "qrc:/image/warning.png"

    height: 22
    width: 30

    anchors.left: imgGps.right
    anchors.leftMargin: 10
    anchors.verticalCenter: parent.verticalCenter
  }
  MouseArea {
    anchors.fill: imgWarning
    hoverEnabled: true

    onClicked: {
      itemHeader.showErrMessage()
    }
  }

  Text {
    anchors.centerIn: parent
    text: parent.title

    font.family: "SimHei"
    font.pointSize: 28
    color: "white"
  }

  Text {
    id: currentTime
    width: parent.width * 0.17
    //height: parent.height
    anchors.right: parent.right
    anchors.rightMargin: 40
    anchors.verticalCenter: parent.verticalCenter

    font.family: "SimHei"
    font.pointSize: 14
    color: "white"

    Timer {
      interval: 1000; running: true; repeat: true
      onTriggered: {
        var currentDate = new Date()
        currentTime.text = currentDate.toLocaleString(Qt.locale(), "yyyy-MM-dd HH:mm:ss")

        DataManager.getInfoList()
        imgWarning.visible = (DataManager.infos.length > 0)
      }
    }
  }
}
