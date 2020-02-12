import QtQuick 2.0

Item {

  property var vehicleImage: "qrc:/image/vehicle_v.png"
  property double vx: 56
  property double vy: 300

  Canvas {
    id: canvasVehicleState
    anchors.fill: parent

    onPaint: {
      var ctx = getContext("2d");
      ctx.save()
      //ctx.clearRect(0, 0, width, height)

      // vehicle
      ctx.drawImage("qrc:/image/vehicle_v.png", vx,  vy, height * 88 / 681.0, height * 135 / 681.0)

      ctx.restore()

      console.log("##########")
    }

  }

  Component.onCompleted: {
    canvasVehicleState.loadImage("qrc:/image/vehicle_v.png")
  }

  onVisibleChanged: {
    if (visible) {
      canvasVehicleState.requestPaint()
    }
  }

}
