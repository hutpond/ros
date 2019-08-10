import QtQuick 2.0
import "../basic" as Basic

Item {

  Rectangle {
    anchors.fill: parent
    color: "#000001"
  }

  Image {
    anchors.fill: parent
    source: "qrc:/image/check.png"
  }

  Image {
    id: imageLog

    source: "qrc:/image/deepblue.png"
    x: 37
    y: 30
    width: 135
    height: 132
  }

  Item{
    id: warning

    y: parent.height * 0.07
    width: parent.width * 0.3
    height: parent.height * 0.1
    anchors.horizontalCenter: parent.horizontalCenter

    Image {

      id: imgWarning
      width: parent.width * 0.1
      anchors.left: parent.left
      anchors.leftMargin: 20
      anchors.verticalCenter: parent.verticalCenter

      source: "qrc:/image/warning.png"
    }

    Text {
      id: name
      anchors.left: imgWarning.right
      anchors.leftMargin: 20
      anchors.verticalCenter: parent.verticalCenter

      verticalAlignment: Text.AlignVCenter
      horizontalAlignment: Text.AlignLeft

      font.family: "SimHei"
      font.pointSize: 40
      color: "red"
      text: qsTr("自检失败")
    }
  }

  Basic.CheckErrorTableView {
    width: parent.width * 0.9
    anchors.horizontalCenter: parent.horizontalCenter

    anchors.top: imageLog.bottom
    anchors.topMargin: 20
    anchors.bottom: checkButton.top
    anchors.bottomMargin: 20

    //color: "transparent"
  }

  Rectangle {
    id: checkButton

    y: parent.height * 0.8
    width: parent.width * 0.25
    height: parent.height * 0.08
    anchors.horizontalCenter: parent.horizontalCenter

    Image {
      anchors.fill: parent
      source: "qrc:/svg/button.svg"
    }
    Text {
      anchors.fill: parent
      verticalAlignment: Text.AlignVCenter
      horizontalAlignment: Text.AlignHCenter

      font.family: "SimHei"
      font.pointSize: 28
      color: "white"
      text: qsTr("重新自检")
    }
  }

  signal reCheck()

  MouseArea {
    anchors.fill: checkButton

    onClicked: {
      parent.reCheck()
    }
  }

}
