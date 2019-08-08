import QtQuick 2.12
import QtQuick.Controls 1.4

Item {

  property double wspider: 15
  property int row: 7
  property int column: 3
  property double wgrid: width / column
  property double hgrid: height / row
  property var clrdeep: "#08FFFFFF"
  property var clrlight: "#08212127"

  property int spaceText: 40
  property double column0: 0.3
  property double column1: 0.3
  property double column2: 0.4

  property var errmsgs: []

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
      font.pointSize: 20

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
      font.pointSize: 20

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
      font.pointSize: 20

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
      width: parent.width * column0
      anchors.left: parent.left
      anchors.leftMargin: spaceText
      color: "white"

      verticalAlignment: Text.AlignVCenter
      horizontalAlignment: Text.AlignHCenter
      font.family: "SimHei"
      font.pointSize: 18

    }
    Text {
      id: grid01

      height: parent.height
      width: parent.width * column1
      anchors.left: grid00.right
      anchors.leftMargin: spaceText
      color: "white"

      verticalAlignment: Text.AlignVCenter
      horizontalAlignment: Text.AlignHCenter
      font.family: "SimHei"
      font.pointSize: 18
    }
    Text {
      id: grid02

      height: parent.height
      width: parent.width * column2
      anchors.left: grid01.right
      anchors.leftMargin: spaceText
      color: "white"

      verticalAlignment: Text.AlignVCenter
      horizontalAlignment: Text.AlignHCenter
      font.family: "SimHei"
      font.pointSize: 18
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
      horizontalAlignment: Text.AlignHCenter
      font.family: "SimHei"
      font.pointSize: 18
    }
    Text {
      id: grid11

      height: parent.height
      width: parent.width * column1 - spaceText
      anchors.left: grid10.right
      anchors.leftMargin: spaceText
      color: "white"

      verticalAlignment: Text.AlignVCenter
      horizontalAlignment: Text.AlignHCenter
      font.family: "SimHei"
      font.pointSize: 18
    }
    Text {
      id: grid12

      height: parent.height
      width: parent.width * column2 - spaceText
      anchors.left: grid11.right
      anchors.leftMargin: spaceText
      color: "white"

      verticalAlignment: Text.AlignVCenter
      horizontalAlignment: Text.AlignHCenter
      font.family: "SimHei"
      font.pointSize: 18
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
      horizontalAlignment: Text.AlignHCenter
      font.family: "SimHei"
      font.pointSize: 18
    }
    Text {
      id: grid21

      height: parent.height
      width: parent.width * column1 - spaceText
      anchors.left: grid20.right
      anchors.leftMargin: spaceText
      color: "white"

      verticalAlignment: Text.AlignVCenter
      horizontalAlignment: Text.AlignHCenter
      font.family: "SimHei"
      font.pointSize: 18
    }
    Text {
      id: grid22

      height: parent.height
      width: parent.width * column2 - spaceText
      anchors.left: grid21.right
      anchors.leftMargin: spaceText
      color: "white"

      verticalAlignment: Text.AlignVCenter
      horizontalAlignment: Text.AlignHCenter
      font.family: "SimHei"
      font.pointSize: 18
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
      horizontalAlignment: Text.AlignHCenter
      font.family: "SimHei"
      font.pointSize: 18
    }
    Text {
      id: grid31

      height: parent.height
      width: parent.width * column1 - spaceText
      anchors.left: grid30.right
      anchors.leftMargin: spaceText
      color: "white"

      verticalAlignment: Text.AlignVCenter
      horizontalAlignment: Text.AlignHCenter
      font.family: "SimHei"
      font.pointSize: 18
    }
    Text {
      id: grid32

      height: parent.height
      width: parent.width * column2 - spaceText
      anchors.left: grid31.right
      anchors.leftMargin: spaceText
      color: "white"

      verticalAlignment: Text.AlignVCenter
      horizontalAlignment: Text.AlignHCenter
      font.family: "SimHei"
      font.pointSize: 18
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
      horizontalAlignment: Text.AlignHCenter
      font.family: "SimHei"
      font.pointSize: 18
    }
    Text {
      id: grid41

      height: parent.height
      width: parent.width * column1 - spaceText
      anchors.left: grid40.right
      anchors.leftMargin: spaceText
      color: "white"
    }
    Text {
      id: grid42

      height: parent.height
      width: parent.width * column2 - spaceText
      anchors.left: grid41.right
      anchors.leftMargin: spaceText
      color: "white"

      verticalAlignment: Text.AlignVCenter
      horizontalAlignment: Text.AlignHCenter
      font.family: "SimHei"
      font.pointSize: 18
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
      horizontalAlignment: Text.AlignHCenter
      font.family: "SimHei"
      font.pointSize: 18
    }
    Text {
      id: grid51

      height: parent.height
      width: parent.width * column1 - spaceText
      anchors.left: grid50.right
      anchors.leftMargin: spaceText
      color: "white"

      verticalAlignment: Text.AlignVCenter
      horizontalAlignment: Text.AlignHCenter
      font.family: "SimHei"
      font.pointSize: 18
    }
    Text {
      id: grid52

      height: parent.height
      width: parent.width * column2 - spaceText
      anchors.left: grid51.right
      anchors.leftMargin: spaceText
      color: "white"

      verticalAlignment: Text.AlignVCenter
      horizontalAlignment: Text.AlignHCenter
      font.family: "SimHei"
      font.pointSize: 18
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

  function addErrMsg(msgs) {
    errmsgs = msgs
    spider.count = msgs.size() / 3
    if (spider.count > 6) {
      spider.visible = true
    }
    else {
      spider.visible = false
    }
    showMsg(0)
  }

  function showMsg(index) {
    if (index < 0 || index >= (spider.count - 6)) {
      spider.visible = false
      return
    }
    spider.index = index

    grid00.text = errmsgs[index * 3]
    grid01.text = errmsgs[index * 3 + 1]
    grid02.text = errmsgs[index * 3 + 2]

    if (spider.count - index > 1) {
      grid10.text = errmsgs[(index + 1) * 3]
      grid11.text = errmsgs[(index + 1) * 3 + 1]
      grid12.text = errmsgs[(index + 1) * 3 + 2]
    }
    if (spider.count - index > 2) {
      grid20.text = errmsgs[(index + 2) * 3]
      grid21.text = errmsgs[(index + 2) * 3 + 1]
      grid22.text = errmsgs[(index + 2) * 3 + 2]
    }
    if (spider.count - index > 3) {
      grid30.text = errmsgs[(index + 3) * 3]
      grid31.text = errmsgs[(index + 3) * 3 + 1]
      grid32.text = errmsgs[(index + 3) * 3 + 2]
    }
    if (spider.count - index > 4) {
      grid40.text = errmsgs[(index + 4) * 3]
      grid41.text = errmsgs[(index + 4) * 3 + 1]
      grid42.text = errmsgs[(index + 4) * 3 + 2]
    }
    if (spider.count - index > 5) {
      grid50.text = errmsgs[(index + 5) * 3]
      grid51.text = errmsgs[(index + 5) * 3 + 1]
      grid52.text = errmsgs[(index + 5) * 3 + 2]
    }


  }

}
