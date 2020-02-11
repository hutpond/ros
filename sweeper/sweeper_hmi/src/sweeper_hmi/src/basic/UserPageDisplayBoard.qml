import QtQuick 2.0
import Sweeper.DataManager 1.0

Item {

  Image {
    id: battery

    anchors.left: parent.left
    anchors.top: parent.top

    width: 27
    height: 21

    source: "qrc:/svg/battery.svg"
  }

  Rectangle {
    id: batteryUsed
    color: "#081A2E"

    property int levelRemained: 48

    anchors.top: battery.top
    anchors.bottom: battery.bottom
    anchors.left: battery.right
    anchors.leftMargin: 20
    anchors.right: parent.right

    Rectangle {
      id: rectUsed
      color: "#0989FF"

      anchors.top: parent.top
      anchors.bottom: parent.bottom
      anchors.left: parent.left
      width: parent.width * batteryUsed.levelRemained / 107
    }

    Text {
      id: name

      anchors.top: parent.top
      anchors.bottom: parent.bottom
      x: rectUsed.x + rectUsed.width + 5

      font.family: "SimHei"
      font.pointSize: 12
      verticalAlignment: Text.AlignVCenter
      color: "white"

      text: batteryUsed.levelRemained + "%"
    }
  }

  function setBatteryLevel(level) {
    if (batteryUsed.visible && Math.abs(batteryUsed.levelRemained - level) >= 1.0) {
      batteryUsed.levelRemained = level
      batteryUsed.update()
    }
  }

  Item {
    id: board

    anchors.top: battery.bottom
    anchors.topMargin: 30
    anchors.bottom: parent.bottom
    anchors.left: battery.left
    anchors.right: batteryUsed.right

    Row {
      spacing: 5
      anchors.fill: parent

      property int imgWidth: (width - spacing * 10) / 9 + 1
      property int addCount: width - imgWidth * 9 - spacing * 10

      Repeater {

        model: ListModel {
          id: modelInfo
          ListElement {name: "qrc:/svg/brake.svg";
            name_off: "qrc:/svg/brake_off.svg"; property_name: "data_suction_status"}
          ListElement {name: "qrc:/svg/positionLamp.svg";
            name_off: "qrc:/svg/positionLamp_off.svg"; property_name: "data_width_light"}
          ListElement {name: "qrc:/svg/highBeam.svg";
            name_off: "qrc:/svg/highBeam_off.svg"; property_name: "data_high_beam_light"}
          ListElement {name: "qrc:/svg/lowBeam.svg";
            name_off: "qrc:/svg/lowBeam_off.svg"; property_name: "data_low_beam_light"}
          ListElement {name: "qrc:/svg/spout.svg";
            name_off: "qrc:/svg/spout_off.svg"; property_name: "data_spout_water"}
          ListElement {name: "qrc:/svg/pallet.svg";
            name_off: "qrc:/svg/pallet_off.svg"; property_name: "data_brush_status"}
          ListElement {name: "qrc:/svg/leftSignalLight.svg";
            name_off: "qrc:/svg/leftSignalLight_off.svg"; property_name: "data_left_light"}
          ListElement {name: "qrc:/svg/rightSignalLight.svg";
            name_off: "qrc:/svg/rightSignalLight_off.svg"; property_name: "data_right_light"}
          ListElement {name: "qrc:/svg/setting.svg";
            name_off: "qrc:/svg/setting.svg"; property_name: "data_suction_status"}
        }

        Image {
          property int status: 1
          width: parent.imgWidth
          height: parent.imgWidth
          anchors.bottom: parent.bottom
          source: name

          MouseArea {
            anchors.fill: parent
            onClicked: {
              parent.status = parent.status == 0 ? 1 : 0
              parent.source = parent.status == 0 ? name_off : name
              var value = parent.status
              DataManager.setProperty(property_name, value)
            }
          }

          Timer {
            id: timer
            interval: 200; running: false; repeat: true
            onTriggered: {
              var value = DataManager.getProperty(property_name)
              if("number" == typeof value){
                parent.status = value
                parent.source = value === 0 ? name_off : name
              }
            }
          }

          onVisibleChanged: {
            if (visible && index < 9) {
              timer.running = true
            }
            else {
              timer.running = false
            }
          }
        }
      }
    }
  }
}
