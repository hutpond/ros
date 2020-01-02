import QtQuick 2.12
import QtQuick.Controls 1.4
import Sweeper.DataManager 1.0
import Sweeper.MsgInfo 1.0

Item {

  property double wspider: 20
  property int row: 7
  property int column: 3
  property int sizeView: row - 2
  property double wgrid: width / column
  property double hgrid: height / row
  property var clrdeep: "#08FFFFFF"
  property var clrlight: "#08212127"

  property int spaceText: 10
  property var columns: [0.27, 0.2, 0.53]

  Column {
    spacing: 1
    anchors.left: parent.left
    anchors.right: spider.left
    height: parent.height

    Repeater {

      model: ListModel {
        id: modelInfo
        ListElement {name: "name"; code: "code"; description: "description"}
      }

      Rectangle {
        width: parent.width
        height: hgrid
        color: index % 2 == 0 ? clrdeep : clrlight
        property var pointSize: index == 0 ? 18 : 12

        Row {
          spacing: 1
          anchors.fill: parent

          Repeater {
            model: 3

            Rectangle {
              width: parent.width * columns[index]
              height: parent.height
              color: clrlight

              Text {
                anchors.fill: parent
                anchors.leftMargin: spaceText
                color: "white"

                verticalAlignment: Text.AlignVCenter
                horizontalAlignment: Text.AlignLeft
                font.family: "SimHei"
                font.pointSize: pointSize

                wrapMode: Text.WordWrap

                text: {
                  if (index == 0) {
                    return name
                  }
                  else if (index == 1) {
                    return code
                  }
                  else {
                    return description
                  }
                } // text
              }  // Text
            }  // Rectangle
          }  // Repeater
        }  // Row
      }  // Rectangle
    }  // Repeater
  }  // Column

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

      y: 0
      width: parent.width
      height: parent.height * 0.25
      radius: width / 2.0
      color: "#0097FF"
    }

    MouseArea {
      anchors.fill: parent
      onClicked: {
        var top = spiderbar.y
        var bottom = spiderbar.y + spiderbar.height
        var index = spider.index;
        if (mouse.y > bottom) {
          index += sizeView
          if (index < spider.count) {
            showMsg(index)
          }
        }
        else if (mouse.y < top) {
          index -= sizeView
          if (index >= 0) {
            showMsg(index)
          }
        }
      }
    }
  }

  function showMsg(index) {
    if (index < 0 || index >= (spider.count)) {
      spider.visible = false
      return
    }
    spider.index = index

    var page = (spider.count % sizeView == 0 ? spider.count / sizeView : spider.count / sizeView + 1)
    page = Math.floor(page)
    if (page > 1) {
      var length = (spider.height - spiderbar.height) / (page - 1)
      var pageIndex = spider.index / sizeView;
      spiderbar.y = length * pageIndex
    }

    modelInfo.clear()
    modelInfo.append({name: "故障名称", code: "故障代码", description: "故障描述"});
    for (var i = index; i < Math.min(index + sizeView, spider.count); ++i) {
      modelInfo.append(
            {name: DataManager.infos[i].name,
              code: DataManager.infos[i].code,
              description: DataManager.infos[i].description});
    }
  }

  onVisibleChanged: {
    if (visible) {
      DataManager.getInfoList();
      spider.count = DataManager.infos.length
      if (spider.count > sizeView) {
        spider.visible = true
      }
      else {
        spider.visible = false
      }
      showMsg(0)
    }
  }

}
