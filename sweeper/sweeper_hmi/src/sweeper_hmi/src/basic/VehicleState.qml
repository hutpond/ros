import QtQuick 2.0
import Sweeper.DataManager 1.0

Item {

  property var vehicleImage: "qrc:/image/vehicle_v.png"
  property double vx: 56
  property double vy: 300
  property var rectVehicleMap: Qt.rect(0, 0, 1, 1)

  Canvas {
    id: canvasVehicleState
    anchors.fill: parent

    onPaint: {
      var ctx = getContext("2d");
      ctx.save()

      // vehicle
      ctx.drawImage("qrc:/image/vehicle_v.png", vx,  vy, height * 88 / 681.0, height * 135 / 681.0)

      // targets
      drawTargets()

      ctx.restore()
    }

    function drawTargets() {
      var targets = DataManager.getTargets()
      for (var i = 0; i < targets.length; ) {
        if ("number" == typeof targets[i]) {
          var count = targets[i];
          var points = [];
          for (var j = 1; j <= count; ++j) {
            var point = ptfToPixel(Qt.point(targets[i + j].x, targets[i + j].y))
            points.push(point)
          }
          drawTarget(points)

          i += (count + 1);
        }
      }
    }

    function drawTarget(points) {
      var ctx = getContext("2d");
      ctx.save()

      ctx.lineWidth = 1
      ctx.strokeStyle = "red"
      ctx.fillStyle = "red"

      for (var i = 0; i < points.length; ++i) {
        if (i === 0) {
          ctx.moveTo(point)
        }
        else {
          ctx.lineTo(point)
        }
      }
      ctx.stroke()
      ctx.closePath()
      ctx.restore()
    }

    function calcMapRect() {
      var widthMap = 7
      var heightMap = widthMap * (height / width)
      var xStart = -1.8

      rectVehicleMap = Qt.rect(xStart, -widthMap / 2.0, heightMap, widthMap)
    }

    function ptfToPixel(point) {
      var ptf = Qt.point(
            width * (rectVehicleMap.y + rectVehicleMap.height - point.y) / rectVehicleMap.height,
            height * (rectVehicleMap.x + rectVehicleMap.width - point.x) / rectVehicleMap.width
            )

      return ptf
    }
  }

  Component.onCompleted: {
    canvasVehicleState.loadImage("qrc:/image/vehicle_v.png")
  }

  onVisibleChanged: {
    if (visible) {
      canvasVehicleState.calcMapRect()
      canvasVehicleState.requestPaint()
    }
  }

}
