import QtQuick 2.0

Item {

  id: item

  enum State {
    None,
    Checked,
    UnChecked
  }
  property string text: ""
  property var color: "white"

  Text {
    id: title
    text: parent.text
    color: parent.color

    font.family: "SimHei"
    font.pointSize: 20
    horizontalAlignment: Text.AlignRight
    verticalAlignment: Text.AlignVCenter

    height: parent.height
    anchors.verticalCenter: parent.verticalCenter
    anchors.left: parent.left
  }

  Image {
    id: imgState
    source: ""

    anchors.verticalCenter: title.verticalCenter
    anchors.left: title.right
    anchors.leftMargin: 20
  }

  function setState(state) {
    if (state === CheckStateBox.None) {
      imgState.source = ""
    }
    else if (state === CheckStateBox.Checked) {
      imgState.source = "qrc:/image/checked.png"
    }
    else {
      imgState.source = "qrc:/image/unchecked.png"
    }
  }

}
