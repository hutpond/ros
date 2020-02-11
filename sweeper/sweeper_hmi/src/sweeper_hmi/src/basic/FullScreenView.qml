import QtQuick 2.0

Item {

  property var points: []
  property bool autoMode: false

  Rectangle {
    id: bkgView
    color: "lightGray"

    anchors.fill: parent
  }

  function setPosition(lon, lat) {
    if (autoMode) {
      points.push(Qt.point(lon, lat))
      console.log("#############  " + points.length)
      update()
    }
  }

  function setAutoMode(mode) {
    autoMode = mode
    if (autoMode) {
      points = []
      console.log("!!!!!!!!!!!!!!!!!!11  " + points.length)
    }
  }

}
