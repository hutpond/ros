import QtQuick 2.0
import "../basic" as Basic

Item {

  Rectangle {
    anchors.fill: parent
    color: "#000001"
  }

  Basic.CheckErrorTableView {
    row: 12
    anchors.fill: parent
  }

}
