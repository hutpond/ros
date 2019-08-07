import QtQuick 2.0

Item {

  Canvas {
    id: canvas
    anchors.fill: parent

    // line
    property double lineLen: width * 0.365
    property double radius: width * 0.135
    // circle
    property double radiusCircle: radius * 0.39
    property double xCircleStart: width / 2 + radiusCircle / Math.sqrt(2)
    property double yCircleStart: height / 2 - radiusCircle / Math.sqrt(2)
    // rotate line and point
    property double xRotLineStart: radiusCircle + 20
    property double xRotLineEnd: radius - 20
    property double xRotPoint: radius * 0.8
    // bar
    property double radiusBarIn: xRotLineStart
    property double radiusBarOut: xRotLineStart + 20

    property int indexSum: 100
    property int indexMax: 2 * indexSum + 40
    property int index: 0

    onPaint: {
      var ctx = getContext("2d");
      ctx.save()
      ctx.clearRect(0, 0, width, height)

      ctx.strokeStyle = "#FF158DEE"

      // line
      ctx.beginPath()
      ctx.lineWidth = 5
      var drawLen = lineLen * Math.min(indexSum, index) / indexSum
      ctx.moveTo(0, height / 2)
      ctx.lineTo(drawLen, height / 2)
      if (index > indexSum) {
        ctx.arc(width / 2, height / 2, radius, -Math.PI,
                -Math.PI - Math.PI * (index - indexSum) / indexSum, true)
      }
      ctx.moveTo(width, height / 2)
      ctx.lineTo(width - drawLen, height / 2)
      if (index > indexSum) {
        ctx.arc(width / 2, height / 2, radius, Math.PI * 2,
                Math.PI * 2 - Math.PI * (index - indexSum) / indexSum, true)
      }
      ctx.stroke()
      ctx.closePath()

      // circle
      ctx.beginPath()
      ctx.lineWidth = 1
      ctx.moveTo(xCircleStart, yCircleStart)
      var angleStart = Math.PI * 7 / 4
      ctx.arc(width / 2, height / 2, radiusCircle, angleStart,
              angleStart - Math.PI * index / indexMax, true)
      ctx.moveTo(xCircleStart, yCircleStart)
      ctx.arc(width / 2, height / 2, radiusCircle, angleStart,
              angleStart + Math.PI * index / indexMax, false)
      ctx.stroke()
      ctx.closePath()

      // rotate line and point
      ctx.beginPath()
      ctx.lineWidth = 1
      ctx.strokeStyle = "#30158DEE"
      ctx.translate(width / 2, height / 2);
      for (var i = 0; i < 6; ++i) {
        var angleRot = Math.PI / 3 * i + Math.PI * 2 * index / indexMax
        ctx.rotate(angleRot)
        ctx.moveTo(xRotLineStart, 0)
        ctx.lineTo(xRotLineEnd, 0)
        ctx.rotate(-angleRot)
      }
      ctx.fillStyle = "#FF158DEE"
      for (i = 0; i < 6; ++i) {
        angleRot = Math.PI / 3 * i + Math.PI / 9 - Math.PI * 2 * index / indexMax
        ctx.rotate(angleRot)
        ctx.translate(xRotPoint, 0)
        ctx.moveTo(0, 0)
        ctx.ellipse(0, 0, 4, 4)
        ctx.stroke()
        ctx.fill()
        ctx.translate(-xRotPoint, 0)
        ctx.rotate(-angleRot)
      }
      ctx.translate(-width / 2, -height / 2);
      ctx.closePath()

      // bar
      ctx.beginPath()
      ctx.lineWidth = 1
      ctx.translate(width / 2, height / 2);
      for (i = 0; i < 3; ++i) {
        var angle = Math.PI * 4 * index / indexMax
        if (i === 0) {
          ctx.strokeStyle = "#70158DEE"
          ctx.fillStyle = "#70158DEE"
        }
        else if (i === 1) {
          ctx.strokeStyle = "#40158DEE"
          ctx.fillStyle = "#40158DEE"
        }
        else {
          ctx.strokeStyle = "#40158DEE"
          ctx.fillStyle = "#40158DEE"
          angle *= -1.0
        }
        angle += i * Math.PI * 2 / 3
        var angleOff = Math.PI / 3.5
        ctx.translate(angle)
        ctx.moveTo(radiusBarIn * Math.cos(angle), radiusBarIn * Math.sin(angle))
        ctx.lineTo(radiusBarOut * Math.cos(angle), radiusBarOut * Math.sin(angle))
        ctx.arc(0, 0, radiusBarOut, angle, angle + angleOff, false)
        ctx.lineTo(radiusBarIn * Math.cos(angle + angleOff), radiusBarIn * Math.sin(angle + angleOff))
        ctx.arc(0, 0, radiusBarIn, angle + angleOff, angle, true)
        ctx.stroke()
        ctx.fill()
        ctx.translate(-angle)
      }
      ctx.translate(-width / 2, -height / 2);
      ctx.stroke()
      ctx.closePath()

      ctx.restore()
    }
  }

  Text {
    id: percent
    property int value: 0

    anchors.centerIn: parent
    horizontalAlignment: Text.AlignRight
    verticalAlignment: Text.AlignVCenter

    color: "#158DEE"
    font.family: "Times"
    font.pointSize: 20

    text: "  0 %"
  }

  Timer {
    id: timer
    interval: 10; running: false; repeat: true
    onTriggered: {
      if (!canvas.visible) {
        return
      }

      canvas.index = (++ canvas.index) % canvas.indexMax
      canvas.requestPaint()

      ++ percent.value;
      if ( (percent.value % 10) == 0) {
        var val = percent.value / 10
        var text = val + " %"
        if (val < 10) {
          text = "  " + text;
          percent.text = text
        }
        else if (val < 100) {
          text = " " + text
          percent.text = text
        }
        else {
          //stop()
        }
      }
    }
  }

  function startTimer() {
    timer.start()
  }
}
