import QtQuick 2.0
import QtQuick.Layouts 1.3

Item {
  id: itemCarLocation

  property var latitude: 31.47123
  property var longitude: 119.58345

  Gears {
    width: parent.width * 0.7
    height: parent.height * 0.04

    anchors.top: parent.top
    anchors.topMargin: height
    anchors.horizontalCenter: parent.horizontalCenter
  }

  Text {
    id: textLatitude

    width: parent.width
    height: parent.height * 0.05

    anchors.bottom: parent.bottom
    anchors.bottomMargin: 30
    horizontalAlignment: Text.AlignHCenter
    verticalAlignment: Text.AlignVCenter

    text: qsTr("纬度: ") + fToS(latitude)
    font.family: "SimHei"
    font.pointSize: 12
    color: "#7E7E7E"
  }

  Text {
    id: textLongitude

    width: textLatitude.width
    height: textLatitude.height

    anchors.bottom: textLatitude.top
    anchors.bottomMargin: 20
    horizontalAlignment: Text.AlignHCenter
    verticalAlignment: Text.AlignVCenter

    text: qsTr("经度: ") + fToS(longitude)
    font.family: "SimHei"
    font.pointSize: 12
    color: "#7E7E7E"
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

  function setPosition(lon, lat) {
    if (textLatitude.visible) {
      if (Math.abs(longitude - lon) > 1e-10) {
        longitude = lon
        textLongitude.update()
      }
      if (Math.abs(latitude - lat) > 1e-10) {
        latitude = lat
        textLatitude.update()
      }
    }
  }
}
