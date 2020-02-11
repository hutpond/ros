import QtQuick 2.0

Item {

  property double velocity: 69
  property double currenKm: 1.2
  property double totalKm: 1024.6
  Rectangle {
    anchors.fill: parent
    color: "#081A2E"
  }

  Text {
    id: textAutoState

    anchors.top: parent.top
    anchors.topMargin: 30
    anchors.left: parent.left
    anchors.leftMargin: 30

    width: 40
    height: 40
    verticalAlignment: Text.AlignVCenter
    horizontalAlignment: Text.AlignHCenter

    text: qsTr("M")
    font.family: "SimHei"
    font.pointSize: 20
    color: "#09A5FF"
  }

  Gears {
    id: gearsUserPage

    anchors.top: parent.top
    anchors.topMargin: 30
    anchors.right: parent.right
    anchors.rightMargin: 30

    width: parent.width * 0.3
    height: textAutoState.height
  }

  Image {
    id: imgVehicle
    source: "qrc:/image/vehicle.png"

    anchors.centerIn: parent
    width: 293
    height: 193
  }

  Text {
    id: textVelocity
    text: velocity

    font.family: "SimHei"
    font.pointSize: 28
    color: "#D4EFFF"

    anchors.left: parent.left
    anchors.leftMargin: 30
    anchors.bottom: parent.bottom
    anchors.bottomMargin: 30

    verticalAlignment: Text.AlignBottom
    horizontalAlignment: Text.AlignRight

    width: 60
    height: 50
  }

  Text {
    id: textVelUnit
    text: qsTr("Km/h")

    font.family: "SimHei"
    font.pointSize: 12
    color: "#D4EFFF"

    anchors.left: textVelocity.right
    anchors.leftMargin: 10
    anchors.top: textVelocity.top
    anchors.bottom: textVelocity.bottom

    verticalAlignment: Text.AlignBottom
    horizontalAlignment: Text.AlignLeft

    width: 40
  }

  Text {
    id: textCurKm
    text: "本次里程: " + currenKm + "Km"

    font.family: "SimHei"
    font.pointSize: 14
    color: "#D4EFFF"

    anchors.top: textVelocity.top
    anchors.bottom: textVelocity.bottom
    anchors.horizontalCenter: parent.horizontalCenter

    verticalAlignment: Text.AlignBottom
    horizontalAlignment: Text.AlignLeft

    width: parent.width * 0.3
  }

  Text {
    id: textTotalKm
    text: "总里程: " + totalKm + "Km"

    font.family: "SimHei"
    font.pointSize: 14
    color: "#D4EFFF"

    anchors.top: textVelocity.top
    anchors.bottom: textVelocity.bottom
    anchors.right: parent.right
    anchors.rightMargin: 30

    verticalAlignment: Text.AlignBottom
    horizontalAlignment: Text.AlignRight

    width: parent.width * 0.3
  }

  function setVehicleGear(value) {
    gearsUserPage.setGearMode(value)
  }

  function setVelocity(value) {
    if (textVelocity.visible && Math.abs(velocity - value) >= 1) {
      velocity = value
      textCurKm.update()
    }
  }

  function setCurrentKm(value) {
    if (textTotalKm.visible && Math.abs(currenKm - value) >= 1) {
      currenKm = value
      textTotalKm.update()
    }
  }

  function setTotalKm(value) {
    if (textVelocity.visible && Math.abs(totalKm - value) >= 1) {
      totalKm = value
      textVelocity.update()
    }
  }

}
