import QtQuick 2.0
import QtQuick.Layouts 1.3
import Sweeper.DataManager 1.0

Item {

  id: dashBoard
  property int spacing: 20

  Row {
    id: firstRow

    width: parent.width
    height: (parent.height - parent.spacing) / 2
    anchors.top: parent.top
    anchors.left: parent.left

    property double itemWidth: Math.min(width / 5.0, height)
    spacing: (width - itemWidth * 5) / 4.0

    Repeater {

      model: ListModel {
        id: modelInfo
        ListElement {name: "qrc:/svg/brake.svg";
          name_off: "qrc:/svg/brake_off.svg"; property_name: "data_suction_status"}
        ListElement {name: "qrc:/svg/leftSignalLight.svg";
          name_off: "qrc:/svg/leftSignalLight_off.svg"; property_name: "data_left_light"}
        ListElement {name: "qrc:/svg/rightSignalLight.svg";
          name_off: "qrc:/svg/rightSignalLight_off.svg"; property_name: "data_right_light"}
        ListElement {name: "qrc:/svg/setting.svg";
          name_off: "qrc:/svg/setting.svg"; property_name: ""}
        ListElement {name: "qrc:/svg/water.svg";
          name_off: "qrc:/svg/water.svg"; property_name: ""}
      }

      Image {
        property int status: 1
        width: parent.itemWidth
        height: parent.itemWidth
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
          if (visible && index < 3) {
            timer.running = true
          }
          else {
            timer.running = false
          }
        }
      }
    }
  }

  Row {
    id: secondRow

    width: firstRow.width
    height: firstRow.height
    anchors.bottom: parent.bottom
    anchors.left: parent.left

    property double itemWidth: firstRow.itemWidth
    spacing: firstRow.spacing

    Repeater {

      model: ListModel {
        id: modelInfo2
        ListElement {name: "qrc:/svg/spout.svg";
          name_off: "qrc:/svg/spout_off.svg"; property_name: "data_spout_water"}
        ListElement {name: "qrc:/svg/pallet.svg";
          name_off: "qrc:/svg/pallet_off.svg"; property_name: "data_brush_status"}
        ListElement {name: "qrc:/svg/positionLamp.svg";
          name_off: "qrc:/svg/positionLamp_off.svg"; property_name: "data_width_light"}
        ListElement {name: "qrc:/svg/highBeam.svg";
          name_off: "qrc:/svg/highBeam_off.svg"; property_name: "data_high_beam_light"}
        ListElement {name: "qrc:/svg/lowBeam.svg";
          name_off: "qrc:/svg/lowBeam_off.svg"; property_name: "data_low_beam_light"}
      }

      Image {
        property int status: 1
        width: parent.itemWidth
        height: parent.itemWidth
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
          id: timer2
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
          if (visible && index < 3) {
            timer2.running = true
          }
          else {
            timer2.running = false
          }
        }
      }
    }
  }
}

