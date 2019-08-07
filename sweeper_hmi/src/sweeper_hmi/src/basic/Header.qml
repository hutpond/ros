import QtQuick 2.0

Item {

  id: itemHeader
  property string title

  Rectangle {
    anchors.fill: parent
    color: "darkGray"
    property alias title: itemHeader.title

    Image {
      id: logPng
      source: "qrc:/image/deepblue.png"

      x: 5
      y: x
      height: parent.height - 2 * y
      width: height
    }

    Text {
      id: currentTime
      x: logPng.x + logPng.width + 20
      width: parent.width * 0.1
      //height: parent.height
      anchors.verticalCenter: parent.verticalCenter

      font.family: "SimHei"
      font.pointSize: 12
      color: "black"

      Timer {
        interval: 1000; running: true; repeat: true
        onTriggered: {
          var currentDate = new Date()
          currentTime.text = currentDate.toLocaleString(Qt.locale(), "yyyy-MM-dd HH:mm:ss")
        }
      }
    }

    Text {
      anchors.centerIn: parent
      text: parent.title

      font.family: "SimHei"
      font.pointSize: 20
      color: "black"
    }
  }
}
