import QtQuick 2.0

Item {
  Image {
    source: "qrc:/image/deepblue.png"
    y: parent.height / 5
    anchors.horizontalCenter: parent.horizontalCenter
  }

  Text {
    text: "深兰AI智能扫路机"
    font.family: "SimHei"
    font.pointSize: 36
    color: "black"

    y: parent.height * 3 / 4
    anchors.horizontalCenter: parent.horizontalCenter
  }
}
