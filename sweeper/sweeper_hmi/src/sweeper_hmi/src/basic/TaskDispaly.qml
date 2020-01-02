import QtQuick 2.12

Item {

  property int topSpace: 20
  property int textHeight: 36
  property int textWidth: width / 2

  Rectangle {
    id: code
    color: "#0A0A0A"
    property alias text: textCode.text

    x: topSpace
    y: 0
    width: textWidth
    height: textHeight

    Text {
      id: textCode
      text: qsTr("text")

      font.family: "SimHei"
      font.pointSize: 14
      horizontalAlignment: Text.AlignLeft
      verticalAlignment: Text.AlignVCenter
      color: "#7E7E7E"
    }
  }

  Rectangle {
    id: task
    color: "#0A0A0A"
    property alias text: textTask.text

    anchors.top: code.top
    anchors.bottom: code.bottom
    anchors.left: code.right
    anchors.right: parent.right

    Text {
      id: textTask
      text: qsTr("text")

      font.family: "SimHei"
      font.pointSize: 14
      horizontalAlignment: Text.AlignLeft
      verticalAlignment: Text.AlignVCenter
      color: "#7E7E7E"
    }
  }

  Rectangle {
    id: line
    color: "#0A0A0A"
    property alias text: textLine.text

    anchors.top: code.bottom
    anchors.left: code.left
    anchors.right: code.right
    height: textHeight

    Text {
      id: textLine
      text: qsTr("text")

      font.family: "SimHei"
      font.pointSize: 14
      horizontalAlignment: Text.AlignLeft
      verticalAlignment: Text.AlignVCenter
      color: "#7E7E7E"
    }
  }

  Rectangle {
    id: map
    color: "#0A0A0A"
    property alias text: textMap.text

    anchors.top: line.bottom
    anchors.left: line.left
    anchors.right: line.right
    height: textHeight

    Text {
      id: textMap
      text: qsTr("text")

      font.family: "SimHei"
      font.pointSize: 14
      horizontalAlignment: Text.AlignLeft
      verticalAlignment: Text.AlignVCenter
      color: "#7E7E7E"
    }
  }

  Rectangle {
    id: address
    color: "#0A0A0A"
    property alias text: textAddress.text

    anchors.top: map.top
    anchors.bottom: map.bottom
    anchors.left: map.right
    anchors.right: parent.right

    Text {
      id: textAddress
      text: qsTr("text")

      font.family: "SimHei"
      font.pointSize: 14
      horizontalAlignment: Text.AlignLeft
      verticalAlignment: Text.AlignVCenter
      color: "#7E7E7E"
    }
  }

  // 清扫统计
  Canvas {
    id: canvasClean

    anchors.left: parent.left
    anchors.right: parent.right
    anchors.top: map.bottom
    anchors.topMargin: 20

    height: parent.height * 0.28

    property double percenMileage: 0.4
    property double percenArea: 0.17

    onPaint:{
      var ctx = canvasClean.getContext('2d');

      ctx.fillStyle = "#081A2E"
      ctx.fillRect(0, 0, width, height)

      var radius = height * 0.35
      var xSpace = (width - 4 * radius) / 4
      var xPos = xSpace
      var yPos = 10
      xPos += radius
      yPos += radius
      drawMileage(ctx, xPos, yPos, radius, percenMileage, "#16A5E3", "#2B7ADE",
                  "#F7B223", "#ED625A", 1, qsTr("清扫里程"), qsTr("非工作里程"))

      xPos += radius * 2 + xSpace * 2
      drawMileage(ctx, xPos, yPos, radius, percenArea, "#16A5E3", "#2B7ADE",
                  "#F7B223", "#ED625A", -1, qsTr("自动清扫面积"), qsTr("人工清扫面积"))
    }

    function drawMileage(ctx, xPos, yPos, radius, percent, clr, clr2, clr3, clr4, sign, name, name2) {
      var radius2 = radius - 20
      var startAngle = Math.PI * (-0.25 - percent)
      if (startAngle < 0) {
        startAngle += 2 * Math.PI
      }
      var endAngle = startAngle - Math.PI * 2.0 * (1.0 - percent)
      if (endAngle < 0) {
        endAngle += 2 * Math.PI
      }

      // left percent
      ctx.beginPath()
      var conical = ctx.createConicalGradient(xPos, yPos, Math.PI * 2 - startAngle)
      conical.addColorStop(0, clr)
      conical.addColorStop(1 - percent, clr2)
      ctx.fillStyle = conical
      ctx.arc(xPos, yPos, radius, startAngle, endAngle, true)
      ctx.lineTo(xPos, yPos)
      ctx.fill()

      // left line
      var length = 20
      var length2 = 45
      ctx.beginPath()
      ctx.strokeStyle = clr2
      ctx.lineWidth = 1
      ctx.moveTo(xPos + radius * Math.sin(Math.PI * 1.75),
                 yPos + radius * Math.cos(Math.PI * 1.75))
      ctx.lineTo(xPos + (radius + length) * Math.sin(Math.PI * 1.75),
                 yPos + (radius + length) * Math.cos(Math.PI * 1.75))
      ctx.lineTo(xPos + (radius + length) * Math.sin(Math.PI * 1.75) - length2,
                 yPos + (radius + length) * Math.cos(Math.PI * 1.75))
      ctx.stroke()

      // left text
      ctx.beginPath()
      ctx.strokeStyle = "#7E7E7E"
      ctx.fillStyle = "#7E7E7E"
      ctx.font = "12px sans-serif"
      ctx.text((1 - percent) * 100 + "%",
               xPos + (radius + length) * Math.sin(Math.PI * 1.75) - 40,
               yPos + (radius + length) * Math.cos(Math.PI * 1.75) - 5)
      ctx.stroke()
      ctx.fill()

      // left inner circle
      ctx.beginPath()
      ctx.fillStyle = "#081A2E"
      ctx.ellipse(xPos - radius2, yPos - radius2, radius2 * 2, radius2 * 2)
      ctx.fill()

      radius += 2 * sign
      radius2 -= 2 * sign
      startAngle = endAngle
      endAngle = startAngle - Math.PI * 2.0 * percent
      if (endAngle < 0) {
        endAngle += 2 * Math.PI
      }

      // right percent
      ctx.beginPath()
      conical = ctx.createConicalGradient(xPos, yPos, Math.PI * 2 - startAngle)
      conical.addColorStop(0, clr3)
      conical.addColorStop(percent, clr4)
      ctx.fillStyle = conical
      ctx.arc(xPos, yPos, radius, startAngle, endAngle, true)
      ctx.lineTo(xPos, yPos)
      ctx.fill()

      // right line
      ctx.beginPath()
      ctx.strokeStyle = clr3
      ctx.lineWidth = 1
      ctx.moveTo(xPos + radius * Math.sin(Math.PI * 0.75),
                 yPos + radius * Math.cos(Math.PI * 0.75))
      ctx.lineTo(xPos + (radius + length) * Math.sin(Math.PI * 0.75),
                 yPos + (radius + length) * Math.cos(Math.PI * 0.75))
      ctx.lineTo(xPos + (radius + length) * Math.sin(Math.PI * 0.75) + length2,
                 yPos + (radius + length) * Math.cos(Math.PI * 0.75))
      ctx.stroke()

      // right text
      ctx.beginPath()
      ctx.strokeStyle = "#7E7E7E"
      ctx.fillStyle = "#7E7E7E"
      ctx.font = "12px sans-serif"
      ctx.text(percent * 100 + "%",
               xPos + (radius + length) * Math.sin(Math.PI * 0.75) + 15,
               yPos + (radius + length) * Math.cos(Math.PI * 0.75) + 12)
      ctx.stroke()
      ctx.fill()

      // right inner circle
      ctx.beginPath()
      ctx.fillStyle = "#081A2E"
      ctx.ellipse(xPos - radius2, yPos - radius2, radius2 * 2, radius2 * 2)
      ctx.fill()

      var textWidth = radius * 1.7
      var textYPos = yPos + radius + 30
      var textXPos = xPos - textWidth
      var textXPos2 = textXPos + fontMetrics.height + 5
      var signWidth = fontMetrics.height / 2

      // left sign
      ctx.beginPath()
      var linear = ctx.createLinearGradient(textXPos + signWidth, textYPos - signWidth,
                                            signWidth, signWidth)
      linear.addColorStop(0, clr)
      linear.addColorStop(percent, clr2)
      ctx.fillStyle = linear
      ctx.fillRect(textXPos + signWidth, textYPos - signWidth,
                   signWidth, signWidth)
      ctx.fill()

      // right sign
      ctx.beginPath()
      linear = ctx.createLinearGradient(textXPos + signWidth + textWidth, textYPos - signWidth,
                                        signWidth, signWidth)
      linear.addColorStop(0, clr3)
      linear.addColorStop(percent, clr4)
      ctx.fillStyle = linear
      ctx.fillRect(textXPos + signWidth + textWidth, textYPos - signWidth,
                   signWidth, signWidth)
      ctx.fill()

      // text
      ctx.beginPath()
      ctx.strokeStyle = "#7E7E7E"
      ctx.fillStyle = "#7E7E7E"
      ctx.font = "12px sans-serif"
      ctx.textAlign = "left"
      ctx.text(name, textXPos2, textYPos)
      ctx.textAlign = "left"
      ctx.text(name2, textXPos2 + textWidth, textYPos)
      ctx.stroke()
      ctx.fill()
    }
  }

  FontMetrics {
    id: fontMetrics
    font.family: "sans-serif"
    font.pixelSize: 12
  }

  // 时间统计
  Canvas {
    id: canvasTime

    anchors.left: parent.left
    anchors.right: parent.right
    anchors.top: canvasClean.bottom
    anchors.topMargin: 20

    height: canvasClean.height

    onPaint:{
      var ctx = canvasTime.getContext('2d');
      ctx.fillStyle = "#081A2E"
      ctx.fillRect(0, 0, width, height)

      // y axis
      var yAxisPosX = width * 0.1
      var yAxisStart = height * 0.1
      var yAxisEnd = height * 0.75

      ctx.beginPath()
      ctx.strokeStyle = "#7E7E7E"
      ctx.lineWidth = 1
      ctx.moveTo(yAxisPosX, yAxisStart)
      ctx.lineTo(yAxisPosX, yAxisEnd)
      ctx.stroke()

      // x axis
      var xAxisPosY = yAxisEnd - 10
      var xAxisStart = yAxisPosX - 10
      var xAxisEnd = width * 0.95

      ctx.beginPath()
      ctx.strokeStyle = "#7E7E7E"
      ctx.lineWidth = 1
      ctx.moveTo(xAxisStart, xAxisPosY)
      ctx.lineTo(xAxisEnd, xAxisPosY)
      ctx.stroke()

      // y value
      var yValueNumber = 6
      var yUnitAxis = (xAxisPosY - yAxisStart) / (yValueNumber - 1)
      for (var i = 0; i < yValueNumber; ++i) {
        ctx.beginPath()
        ctx.strokeStyle = "#7E7E7E"
        ctx.fillStyle = "#7E7E7E"
        ctx.font = "12px sans-serif"
        ctx.textAlign = "right"
        ctx.text((yValueNumber - i - 1) * 20, yAxisPosX - 30, yAxisStart + i * yUnitAxis + 5)
        ctx.stroke()
        ctx.fill()
      }

      // x value
      var xValueNumber = 10
      var xUnitAxis = (xAxisEnd - yAxisPosX) / (xValueNumber - 1)
      for (i = 0; i < xValueNumber; ++i) {
        ctx.beginPath()
        ctx.strokeStyle = "#7E7E7E"
        ctx.fillStyle = "#7E7E7E"
        ctx.font = "12px sans-serif"
        ctx.textAlign = "center"
        var text = 12 + parseInt(i / 2)
        text += ":"
        text += i === 0 ? "00" : "30"
        ctx.text(text, xAxisStart + xUnitAxis * i - 7, xAxisPosY + 20)
        ctx.stroke()
        ctx.fill()
      }

      // times
      var signHeight = 14
      var signWidth = xUnitAxis * 7
      var signX = xAxisStart + xUnitAxis + 7
      var signY = yAxisStart + 3 * yUnitAxis + 5
      ctx.beginPath()
      var linear = ctx.createLinearGradient(signX, signY, signWidth, signWidth)
      linear.addColorStop(0, "#16A5E3")
      linear.addColorStop(1, "#2B7ADE")
      ctx.fillStyle = linear
      ctx.fillRect(signX, signY, signWidth, signHeight)
      ctx.fill()

      signWidth = xUnitAxis
      signY = yAxisStart + 2 * yUnitAxis + 5
      ctx.beginPath()
      linear = ctx.createLinearGradient(signX, signY, signWidth, signWidth)
      linear.addColorStop(0, "#A8E34C")
      linear.addColorStop(1, "#5AA62B")
      ctx.fillStyle = linear
      ctx.fillRect(signX, signY, signWidth, signHeight)
      ctx.fill()

      signWidth = xUnitAxis * 2
      signX = xAxisStart + 2 * xUnitAxis + 7
      signY = yAxisStart + yUnitAxis + 5
      ctx.beginPath()
      linear = ctx.createLinearGradient(signX, signY, signWidth, signWidth)
      linear.addColorStop(0, "#F7B223")
      linear.addColorStop(1, "#ED625A")
      ctx.fillStyle = linear
      ctx.fillRect(signX, signY, signWidth, signHeight)
      ctx.fill()

      // text
      var textX = yAxisPosX
      var textY = xAxisPosY + 35
      var textHeight = fontMetrics.height / 2
      ctx.beginPath()
      linear = ctx.createLinearGradient(textX, textY, textHeight, textHeight)
      linear.addColorStop(0, "#16A5E3")
      linear.addColorStop(1, "#2B7ADE")
      ctx.fillStyle = linear
      ctx.fillRect(textX, textY, textHeight, textHeight)
      ctx.fill()

      ctx.beginPath()
      ctx.strokeStyle = "#7E7E7E"
      ctx.fillStyle = "#7E7E7E"
      ctx.font = "12px sans-serif"
      ctx.textAlign = "left"
      ctx.text(qsTr("工作时段"), textX + textHeight * 1.5, textY + textHeight)
      ctx.stroke()
      ctx.fill()

      // text 2
      textX += width * 0.15
      ctx.beginPath()
      linear = ctx.createLinearGradient(textX, textY, textHeight, textHeight)
      linear.addColorStop(0, "#A8E34C")
      linear.addColorStop(1, "#5AA62B")
      ctx.fillStyle = linear
      ctx.fillRect(textX, textY, textHeight, textHeight)
      ctx.fill()

      ctx.beginPath()
      ctx.strokeStyle = "#7E7E7E"
      ctx.fillStyle = "#7E7E7E"
      ctx.font = "12px sans-serif"
      ctx.textAlign = "left"
      ctx.text(qsTr("清扫时段"), textX + textHeight * 1.5, textY + textHeight)
      ctx.stroke()
      ctx.fill()

      // text 3
      textX += width * 0.15
      ctx.beginPath()
      linear = ctx.createLinearGradient(textX, textY, textHeight, textHeight)
      linear.addColorStop(0, "#F7B223")
      linear.addColorStop(1, "#ED625A")
      ctx.fillStyle = linear
      ctx.fillRect(textX, textY, textHeight, textHeight)
      ctx.fill()

      ctx.beginPath()
      ctx.strokeStyle = "#7E7E7E"
      ctx.fillStyle = "#7E7E7E"
      ctx.font = "12px sans-serif"
      ctx.textAlign = "left"
      ctx.text(qsTr("自动清扫时段"), textX + textHeight * 1.5, textY + textHeight)
      ctx.stroke()
      ctx.fill()
    }
  }

  // 项目统计
  property int itemSpace: 20
  Rectangle {
    id: itemPower
    color: "#081A2E"

    anchors.left: parent.left
    anchors.top: canvasTime.bottom
    anchors.topMargin: 20
    anchors.bottom: parent.bottom
    anchors.bottomMargin: 20

    width: (parent.width - 2 * itemSpace) / 3

    Image {
      id: imagePower

      anchors.left: parent.left
      anchors.top: parent.top
      anchors.bottom: parent.bottom
      anchors.leftMargin: 10
      anchors.topMargin: 10
      anchors.bottomMargin: 10

      width: height

      source: "qrc:/svg/task_power.svg"
    }

    Text {
      id: textPowerName

      anchors.top: imagePower.top
      anchors.left: imagePower.right
      anchors.right: parent.right
      anchors.leftMargin: 10

      height: imagePower.height * 0.3

      font.family: "SimHei"
      font.pointSize: 12
      horizontalAlignment: Text.AlignLeft
      verticalAlignment: Text.AlignVCenter
      color: "#7E7E7E"

      text: qsTr("任务用电")
    }

    Text {
      id: textPowerValue

      anchors.top: textPowerName.bottom
      anchors.bottom: imagePower.bottom
      anchors.left: textPowerName.left

      width: metricsPowerValue.width * 1.5

      font.family: metricsPowerValue.font.family
      font.pointSize: metricsPowerValue.font.pixelSize
      horizontalAlignment: Text.AlignLeft
      verticalAlignment: Text.AlignBottom
      color: "#7E7E7E"

      text: metricsPowerValue.text
    }

    TextMetrics {
      id: metricsPowerValue

      font.family: "SimHei"
      font.pixelSize: 24

      text: qsTr("10")
    }

    Text {
      id: textPowerUnit

      anchors.top: textPowerValue.top
      anchors.bottom: textPowerValue.bottom
      anchors.left: textPowerValue.right
      anchors.right: imagePower.right

      font.family: "SimHei"
      font.pointSize: 16
      horizontalAlignment: Text.AlignLeft
      verticalAlignment: Text.AlignBottom
      color: "#7E7E7E"

      text: qsTr("kW·h")
    }
  }
  Rectangle {
    id: itemWater
    color: "#081A2E"

    anchors.left: itemPower.right
    anchors.leftMargin: itemSpace
    anchors.top: itemPower.top
    anchors.bottom: itemPower.bottom

    width: itemPower.width

    Image {
      id: imageWater

      anchors.left: parent.left
      anchors.top: parent.top
      anchors.bottom: parent.bottom
      anchors.leftMargin: 10
      anchors.topMargin: 10
      anchors.bottomMargin: 10

      width: height

      source: "qrc:/svg/task_water.svg"
    }

    Text {
      id: textWaterName

      anchors.top: imageWater.top
      anchors.left: imageWater.right
      anchors.right: parent.right
      anchors.leftMargin: 10

      height: imageWater.height * 0.3

      font.family: "SimHei"
      font.pointSize: 12
      horizontalAlignment: Text.AlignLeft
      verticalAlignment: Text.AlignVCenter
      color: "#7E7E7E"

      text: qsTr("任务用水")
    }

    Text {
      id: textWaterValue

      anchors.top: textWaterName.bottom
      anchors.bottom: imageWater.bottom
      anchors.left: textWaterName.left

      width: metricsWaterValue.width * 1.5

      font.family: metricsWaterValue.font.family
      font.pointSize: metricsWaterValue.font.pixelSize
      horizontalAlignment: Text.AlignLeft
      verticalAlignment: Text.AlignBottom
      color: "#7E7E7E"

      text: metricsWaterValue.text
    }

    TextMetrics {
      id: metricsWaterValue

      font.family: "SimHei"
      font.pixelSize: 24

      text: qsTr("50")
    }

    Text {
      id: textWaterUnit

      anchors.top: textWaterValue.top
      anchors.bottom: textWaterValue.bottom
      anchors.left: textWaterValue.right
      anchors.right: imageWater.right

      font.family: "SimHei"
      font.pointSize: 16
      horizontalAlignment: Text.AlignLeft
      verticalAlignment: Text.AlignBottom
      color: "#7E7E7E"

      text: qsTr("L")
    }
  }
  Rectangle {
    id: itemGarbage
    color: "#081A2E"

    anchors.left: itemWater.right
    anchors.leftMargin: itemSpace
    anchors.top: itemWater.top
    anchors.bottom: itemWater.bottom

    width: itemPower.width

    Image {
      id: imageGarbage

      anchors.left: parent.left
      anchors.top: parent.top
      anchors.bottom: parent.bottom
      anchors.leftMargin: 10
      anchors.topMargin: 10
      anchors.bottomMargin: 10

      width: height

      source: "qrc:/svg/task_garbage.svg"
    }

    Text {
      id: textGarbageName

      anchors.top: imageGarbage.top
      anchors.left: imageGarbage.right
      anchors.right: parent.right
      anchors.leftMargin: 10

      height: imageGarbage.height * 0.3

      font.family: "SimHei"
      font.pointSize: 12
      horizontalAlignment: Text.AlignLeft
      verticalAlignment: Text.AlignVCenter
      color: "#7E7E7E"

      text: qsTr("清扫垃圾")
    }

    Text {
      id: textGarbageValue

      anchors.top: textGarbageName.bottom
      anchors.bottom: imageGarbage.bottom
      anchors.left: textGarbageName.left

      width: metricsGarbageValue.width * 1.5

      font.family: metricsGarbageValue.font.family
      font.pointSize: metricsGarbageValue.font.pixelSize
      horizontalAlignment: Text.AlignLeft
      verticalAlignment: Text.AlignBottom
      color: "#7E7E7E"

      text: metricsGarbageValue.text
    }

    TextMetrics {
      id: metricsGarbageValue

      font.family: "SimHei"
      font.pixelSize: 24

      text: qsTr("25")
    }

    Text {
      id: textGarbageUnit

      anchors.top: textGarbageValue.top
      anchors.bottom: textGarbageValue.bottom
      anchors.left: textGarbageValue.right
      anchors.right: imageGarbage.right

      font.family: "SimHei"
      font.pointSize: 16
      horizontalAlignment: Text.AlignLeft
      verticalAlignment: Text.AlignBottom
      color: "#7E7E7E"

      text: qsTr("kg")
    }
  }

  onVisibleChanged: {
    if (visible) {
      canvasClean.requestPaint()
      canvasTime.requestPaint()
    }
  }
}
