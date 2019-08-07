import QtQuick 2.0
import QtQuick.Layouts 1.3

Item {
  id: itemCarLocation

  property var latitude: 31.47123
  property var longitude: 119.58345

  Rectangle {
    anchors.fill: parent
    color: "lightGray"
  }

  ColumnLayout {
    property alias latitude: itemCarLocation.latitude
    property alias longitude: itemCarLocation.longitude

    width: parent.width
    height: parent.height * 0.08
    x: 0
    y: parent.height - height - 5

    Text {
      //anchors.horizontalCenter: parent.horizontalCenter
      text: qsTr("经度: ") + fToS(longitude)

      font.family: "SimHei"
      font.pointSize: 12
      color: "black"
    }

    Text {
//      anchors.horizontalCenter: parent.horizontalCenter
      text: qsTr("纬度: ") + fToS(latitude)

      font.family: "SimHei"
      font.pointSize: 12
      color: "black"
    }
  }

  function fToS(value) {
    var ret = ""

    var valDegree = Math.floor(Math.abs(value));
    var valMinute = (Math.abs(value) - valDegree) * 60
    var valSeconds = (valMinute - Math.floor(valMinute)) * 60
    valMinute = Math.floor(valMinute)

    if (valDegree < 100 && valDegree >= 10) {
      ret += "  "
    }
    else if(valDegree < 10) {
      ret += "    "
    }
    if (value < 0) begin += "-"
    else ret += " "
    ret += valDegree + "°"

    if (valMinute < 10) {
      ret += "  "
    }
    ret += valMinute + "′"

    if (valSeconds < 10) {
      ret += "  "
    }
    ret += valSeconds.toFixed(2) + "″"

    return ret
  }
}
