import QtQuick 2.12
import QtQuick.Controls 1.4
import Sweeper.DataManager 1.0
import Sweeper.MsgInfo 1.0

Item {

  property double wspider: 15
  property int row: 7
  property int column: 3
  property double wgrid: width / column
  property double hgrid: height / row
  property var clrdeep: "#08FFFFFF"
  property var clrlight: "#08212127"

  property int spaceText: 10
  property double column0: 0.27
  property double column1: 0.2
  property double column2: 0.53

  Rectangle {
    id: rowheader

    height: hgrid
    anchors.top: parent.top
    anchors.left: parent.left
    anchors.right: spider.left
    color: clrdeep

    Text {
      id: header0

      height: parent.height
      width: parent.width * column0 - spaceText
      anchors.left: parent.left
      anchors.leftMargin: spaceText
      color: "white"

      verticalAlignment: Text.AlignVCenter
      horizontalAlignment: Text.AlignLeft
      font.family: "SimHei"
      font.pointSize: 18

      text: "故障名称"
    }
    Text {
      id: header1

      height: parent.height
      width: parent.width * column1 - spaceText
      anchors.left: header0.right
      anchors.leftMargin: spaceText
      color: "white"

      verticalAlignment: Text.AlignVCenter
      horizontalAlignment: Text.AlignLeft
      font.family: "SimHei"
      font.pointSize: 18

      text: "故障代码"
    }
    Text {
      id: header2

      height: parent.height
      width: parent.width * column2 - spaceText
      anchors.left: header1.right
      anchors.leftMargin: spaceText
      color: "white"

      verticalAlignment: Text.AlignVCenter
      horizontalAlignment: Text.AlignLeft
      font.family: "SimHei"
      font.pointSize: 18

      text: "故障描述"
    }
  }
  Rectangle {
    id: row0

    height: hgrid
    anchors.top: rowheader.bottom
    anchors.left: parent.left
    anchors.right: spider.left
    color: clrlight

    Text {
      id: grid00

      height: parent.height
      width: parent.width * column0 - spaceText
      anchors.left: parent.left
      anchors.leftMargin: spaceText
      color: "white"

      verticalAlignment: Text.AlignVCenter
      horizontalAlignment: Text.AlignLeft
      font.family: "SimHei"
      font.pointSize: 14

    }
    Text {
      id: grid01

      height: parent.height
      width: parent.width * column1 - spaceText
      anchors.left: grid00.right
      anchors.leftMargin: spaceText
      color: "white"

      verticalAlignment: Text.AlignVCenter
      horizontalAlignment: Text.AlignLeft
      font.family: "SimHei"
      font.pointSize: 14
    }
    Text {
      id: grid02

      height: parent.height
      width: parent.width * column2 - spaceText
      anchors.left: grid01.right
      anchors.leftMargin: spaceText
      color: "white"

      verticalAlignment: Text.AlignVCenter
      horizontalAlignment: Text.AlignLeft
      font.family: "SimHei"
      font.pointSize: 14
    }
  }
  Rectangle {
    id: row1

    height: hgrid
    anchors.top: row0.bottom
    anchors.left: parent.left
    anchors.right: spider.left
    color: clrdeep

    Text {
      id: grid10

      height: parent.height
      width: parent.width * column0 - spaceText
      anchors.left: parent.left
      anchors.leftMargin: spaceText
      color: "white"

      verticalAlignment: Text.AlignVCenter
      horizontalAlignment: Text.AlignLeft
      font.family: "SimHei"
      font.pointSize: 14
    }
    Text {
      id: grid11

      height: parent.height
      width: parent.width * column1 - spaceText
      anchors.left: grid10.right
      anchors.leftMargin: spaceText
      color: "white"

      verticalAlignment: Text.AlignVCenter
      horizontalAlignment: Text.AlignLeft
      font.family: "SimHei"
      font.pointSize: 14
    }
    Text {
      id: grid12

      height: parent.height
      width: parent.width * column2 - spaceText
      anchors.left: grid11.right
      anchors.leftMargin: spaceText
      color: "white"

      verticalAlignment: Text.AlignVCenter
      horizontalAlignment: Text.AlignLeft
      font.family: "SimHei"
      font.pointSize: 14
    }
  }
  Rectangle {
    id: row2

    height: hgrid
    anchors.top: row1.bottom
    anchors.left: parent.left
    anchors.right: spider.left
    color: clrlight

    Text {
      id: grid20

      height: parent.height
      width: parent.width * column0 - spaceText
      anchors.left: parent.left
      anchors.leftMargin: spaceText
      color: "white"

      verticalAlignment: Text.AlignVCenter
      horizontalAlignment: Text.AlignLeft
      font.family: "SimHei"
      font.pointSize: 14
    }
    Text {
      id: grid21

      height: parent.height
      width: parent.width * column1 - spaceText
      anchors.left: grid20.right
      anchors.leftMargin: spaceText
      color: "white"

      verticalAlignment: Text.AlignVCenter
      horizontalAlignment: Text.AlignLeft
      font.family: "SimHei"
      font.pointSize: 14
    }
    Text {
      id: grid22

      height: parent.height
      width: parent.width * column2 - spaceText
      anchors.left: grid21.right
      anchors.leftMargin: spaceText
      color: "white"

      verticalAlignment: Text.AlignVCenter
      horizontalAlignment: Text.AlignLeft
      font.family: "SimHei"
      font.pointSize: 14
    }
  }
  Rectangle {
    id: row3

    height: hgrid
    anchors.top: row2.bottom
    anchors.left: parent.left
    anchors.right: spider.left
    color: clrdeep

    Text {
      id: grid30

      height: parent.height
      width: parent.width * column0 - spaceText
      anchors.left: parent.left
      anchors.leftMargin: spaceText
      color: "white"

      verticalAlignment: Text.AlignVCenter
      horizontalAlignment: Text.AlignLeft
      font.family: "SimHei"
      font.pointSize: 14
    }
    Text {
      id: grid31

      height: parent.height
      width: parent.width * column1 - spaceText
      anchors.left: grid30.right
      anchors.leftMargin: spaceText
      color: "white"

      verticalAlignment: Text.AlignVCenter
      horizontalAlignment: Text.AlignLeft
      font.family: "SimHei"
      font.pointSize: 14
    }
    Text {
      id: grid32

      height: parent.height
      width: parent.width * column2 - spaceText
      anchors.left: grid31.right
      anchors.leftMargin: spaceText
      color: "white"

      verticalAlignment: Text.AlignVCenter
      horizontalAlignment: Text.AlignLeft
      font.family: "SimHei"
      font.pointSize: 14
    }
  }
  Rectangle {
    id: row4

    height: hgrid
    anchors.top: row3.bottom
    anchors.left: parent.left
    anchors.right: spider.left
    color: clrlight

    Text {
      id: grid40

      height: parent.height
      width: parent.width * column0 - spaceText
      anchors.left: parent.left
      anchors.leftMargin: spaceText
      color: "white"

      verticalAlignment: Text.AlignVCenter
      horizontalAlignment: Text.AlignLeft
      font.family: "SimHei"
      font.pointSize: 14
    }
    Text {
      id: grid41

      height: parent.height
      width: parent.width * column1 - spaceText
      anchors.left: grid40.right
      anchors.leftMargin: spaceText
      color: "white"

      verticalAlignment: Text.AlignVCenter
      horizontalAlignment: Text.AlignLeft
      font.family: "SimHei"
      font.pointSize: 14
    }
    Text {
      id: grid42

      height: parent.height
      width: parent.width * column2 - spaceText
      anchors.left: grid41.right
      anchors.leftMargin: spaceText
      color: "white"

      verticalAlignment: Text.AlignVCenter
      horizontalAlignment: Text.AlignLeft
      font.family: "SimHei"
      font.pointSize: 14
    }
  }
  Rectangle {
    id: row5

    height: hgrid
    anchors.top: row4.bottom
    anchors.left: parent.left
    anchors.right: spider.left
    color: clrdeep

    Text {
      id: grid50

      height: parent.height
      width: parent.width * column0 - spaceText
      anchors.left: parent.left
      anchors.leftMargin: spaceText
      color: "white"

      verticalAlignment: Text.AlignVCenter
      horizontalAlignment: Text.AlignLeft
      font.family: "SimHei"
      font.pointSize: 14
    }
    Text {
      id: grid51

      height: parent.height
      width: parent.width * column1 - spaceText
      anchors.left: grid50.right
      anchors.leftMargin: spaceText
      color: "white"

      verticalAlignment: Text.AlignVCenter
      horizontalAlignment: Text.AlignLeft
      font.family: "SimHei"
      font.pointSize: 14
    }
    Text {
      id: grid52

      height: parent.height
      width: parent.width * column2 - spaceText
      anchors.left: grid51.right
      anchors.leftMargin: spaceText
      color: "white"

      verticalAlignment: Text.AlignVCenter
      horizontalAlignment: Text.AlignLeft
      font.family: "SimHei"
      font.pointSize: 14
    }
  }

  Rectangle {
    id: spider

    property int index: 0
    property int count: 1

    width: wspider
    height: parent.height
    anchors.right: parent.right

    color: "#2E3031"
    visible: false

    Rectangle {
      id: spiderbar

      y: (parent.height - height) * spider.index / spider.count
      width: parent.width
      height: parent.height * 0.25
      radius: width / 2.0
      color: "#0097FF"
    }
  }

  function showMsg(index) {
    if (index < 0 || index >= (spider.count - 6)) {
      spider.visible = false
      return
    }
    spider.index = index

    grid00.text = dataManager.infos[index].name
    grid01.text = dataManager.infos[index].code
    grid02.text = dataManager.infos[index].description

    if (spider.count - index > 1) {
      grid10.text = dataManager.infos[index + 1].name
      grid11.text = dataManager.infos[index + 1].code
      grid12.text = dataManager.infos[index + 1].description
    }
    if (spider.count - index > 2) {
      grid20.text = dataManager.infos[index + 2].name
      grid21.text = dataManager.infos[index + 2].code
      grid22.text = dataManager.infos[index + 2].description
    }
    if (spider.count - index > 3) {
      grid30.text = dataManager.infos[index + 3].name
      grid31.text = dataManager.infos[index + 3].code
      grid32.text = dataManager.infos[index + 3].description
    }
    if (spider.count - index > 4) {
      grid40.text = dataManager.infos[index + 4].name
      grid41.text = dataManager.infos[index + 4].code
      grid42.text = dataManager.infos[index + 4].description
    }
    if (spider.count - index > 5) {
      grid50.text = dataManager.infos[index + 5].name
      grid51.text = dataManager.infos[index + 5].code
      grid52.text = dataManager.infos[index + 5].description
    }
  }

  DataManager {
    id: dataManager
  }

  onVisibleChanged: {
    if (visible) {
      spider.count = dataManager.infos.length
      if (spider.count > 6) {
        spider.visible = true
      }
      else {
        spider.visible = false
      }
      showMsg(0)
    }
  }

}
