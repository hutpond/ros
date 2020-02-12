import QtQuick 2.0
import Sweeper.DataManager 1.0

Item {

  property var origin: Qt.point(0, 0)
  property var points: []
  property bool autoMode: false

  property var rectEnu: Qt.rect(0, 0, 1, 1)

  Canvas {
    id: canvasFullView

    anchors.fill: parent

    onPaint: {
      var ctx = getContext("2d");
      ctx.save()

      ctx.fillStyle = "lightGray";
      ctx.fillRect(0, 0, width, height);

      if (points.length > 0) {
        ctx.beginPath()

        for (var i = 0; i < points.length; ++i) {
          var point = enuToPixel(points[i])
          if (i === 0) {
            ctx.lineWidth = 1
            ctx.strokeStyle = "green"
            ctx.fillStyle = "green"
            ctx.moveTo(point)
            ctx.ellipse(point.x, point.y, 4, 4)
            ctx.fill()
          }
          else {
            ctx.lineWidth = 2
            ctx.fillStyle = "blue"
            ctx.lineTo(point)
          }
          if (i === points.length - 1) {
            ctx.lineWidth = 1
            ctx.strokeStyle = "red"
            ctx.fillStyle = "red"
            ctx.ellipse(0, 0, 4, 4)
            ctx.fill()
          }
        }
        ctx.stroke()
        ctx.closePath()
      }

      ctx.restore()
    }

    function enuToPixel(enu) {
      var pixel = Qt.point(
            width * (enu.x - rectEnu.x) / rectEnu.width,
            height * (rectEnu.y + rectEnu.height - enu.y) / rectEnu.height
            )
      return pixel
    }
  }

  onVisibleChanged: {
    if (visible) {
      canvasFullView.requestPaint()
    }
  }

  function setPosition(lon, lat) {
    if (autoMode) {
      if (points.length == 0) {
        origin = Qt.point(lon, lat)
      }

      var lla = Qt.point(lon, lat)
      var enu = DataManager.llaToEnu(lla, origin)

      if (points.length == 0 || Math.sqrt( (enu.x - points[0].x) * (enu.x - points[0].x)
                                          + (enu.y - points[0].y) * (enu.y - points[0].y) ) >= 0.5) {
        points.push(enu)
        if (enu.x < rectEnu.x || enu.x > rectEnu.x + rectEnu.width ||
            enu.y < rectEnu.y || enu.y > rectEnu.y + rectEnu.height) {
          calcsize()
        }

        canvasFullView.requestPaint()
      }
    }
  }

  function setAutoMode(mode) {
    autoMode = mode
    if (autoMode) {
      points = []

      var widthEnu = 5
      var heightEnu = widthEnu * (height / width)
      rectEnu = Qt.rect(-widthEnu / 2.0, -heightEnu / 2.0, widthEnu, heightEnu)
    }
  }

  function calcSize() {
    var xMin = 1000
    var xMax = -1000
    var yMin = 1000
    var yMax = -1000
    for(var i = 0; i < points.length; ++i) {
      if (xMin > points[i].x) {
        xMin = points[i].x
      }
      if (xMax < points[i].x) {
        xMax = points[i].x
      }
      if (yMin > points[i].y) {
        yMin = points[i].y
      }
      if (yMax < points[i].y) {
        yMax = points[i].y
      }
    }
    var xLength = 1.3 * (xMax - xMin)
    var yLength = 1.3 * (yMax - yMin)
    if (xLength / yLength > width / height) {
      yLength = xLength * (height / width)
    }
    else {
      xLength = yLength * (width / height)
    }

    rectEnu = Qt.rect(-xLength / 2.0, -yLength / 2.0, xLength, yLength)
  }

}
