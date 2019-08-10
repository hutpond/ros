import QtQuick 2.0

Item {

  property double vx: 56
  property double vy: 300

  Image {
    id: vehicle
    source: "qrc:/image/vehicle_v.png"

    x: vx
    y: vy
    width: parent.height * 88 / 681.0
    height: parent.height * 135 / 681.0
  }

}
