import QtQuick 2.0

Item {
  Image {
    source: "qrc:/image/deepblue.png"
    x: (parent.width - this.width) / 2
    y: parent.height / 5
  }

  Text {
    text: "深兰AI智能扫路机"
    font.family: "SimHei"
    font.pointSize: 36
    color: "black"

    x: (parent.width - this.width) / 2
    y: parent.height * 3 / 4
  }
}
