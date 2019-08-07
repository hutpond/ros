import QtQuick 2.0
import QtWebView 1.1
import "../basic" as Basic

Item {

  Rectangle {
    anchors.fill: parent
    color: "black"
  }

  Image {
    source: "qrc:/image/deepblue.png"
    x: 37
    y: 30
    width: 135
    height: 132
  }

  Basic.Particles {
    anchors.fill: parent
    visible: true
  }

  Text {
    text: "深兰AI智能扫路机"
    font.family: "SimHei"
    font.pointSize: 36
    color: "white"

    y: 250
    anchors.horizontalCenter: parent.horizontalCenter
  }
}
