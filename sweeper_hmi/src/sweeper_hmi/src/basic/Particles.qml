import QtQuick 2.0
import QtQml 2.12

Item {

  Canvas {
    id: canvas
    anchors.fill: parent

    property var vertices: []

    property int vertexCount: 7000
    property int vertexSize: 3
    property double oceanWidth: 204.0
    property double oceanHeight: -80.0
    property int gridSize: 32
    property int waveSize: 16
    property int perspective: 100

    property int  frame: 0

    onPaint: {

      if (frame === 0) {
        createParticles()
        timer.start()
      }

      var depth = (vertexCount / oceanWidth * gridSize)

      var ctx = getContext("2d");
      ctx.save()
      ctx.clearRect(0, 0, width, height)
      //ctx.fillStyle = `hsl(200deg, 100%, 2%)`
      ctx.fillStyle = Qt.hsla(200 / 360.0, 0.5, 0.5, 1.0)

      var rad = Math.sin(frame / 100) * Math.PI / 20
      var rad2 = Math.sin(frame / 50) * Math.PI / 10
      frame++

      ctx.translate(width / 2, height / 2)

      ctx.beginPath()

      for (var i = 0; i < vertexCount; ++i) {
        var ni = i + oceanWidth
        var x = vertices[i].x - frame % (gridSize * 2)
        var z = vertices[i].z - frame * 2 % gridSize + (i % 2 === 0 ? gridSize / 2 : 0)
        var wave = (Math.cos(frame / 45 + x / 50) - Math.sin(frame / 20 + z / 50) + Math.sin(frame / 30 + z*x / 10000))
        var y = vertices[i].y + wave * waveSize
        var a = Math.max(0, 1 - (Math.sqrt(x ** 2 + z ** 2)) / depth)
        var tx, ty, tz

        y -= oceanHeight

        // Transformation variables
        tx = x
        ty = y
        tz = z

        // Rotation Y
        tx = x * Math.cos(rad) + z * Math.sin(rad)
        tz = -x * Math.sin(rad) + z * Math.cos(rad)

        x = tx
        y = ty
        z = tz

        // Rotation Z
        tx = x * Math.cos(rad) - y * Math.sin(rad)
        ty = x * Math.sin(rad) + y * Math.cos(rad)

        x = tx;
        y = ty;
        z = tz;

        // Rotation X

        ty = y * Math.cos(rad2) - z * Math.sin(rad2)
        tz = y * Math.sin(rad2) + z * Math.cos(rad2)

        x = tx;
        y = ty;
        z = tz;

        x /= z / perspective
        y /= z / perspective


        if (a < 0.01) continue
        if (z < 0) continue

        ctx.fillStyle = Qt.hsla((180 + wave * 20) / 360.0, 1.0, 0.5, a)
        ctx.fillRect(x - a * vertexSize / 2, y - a * vertexSize / 2, a * vertexSize, a * vertexSize)

        ctx.stroke()
        ctx.closePath()
      }

      ctx.restore()
    }

    Timer {
      id: timer
      interval: 30; running: false; repeat: true
      onTriggered: {
        if (!canvas.visible) {
          return
        }

        canvas.requestPaint()
      }
    }

    function createParticles() {
      for (var i = 0; i < vertexCount; i++) {
        var x = i % oceanWidth
        var y = 0
        var z = i / oceanWidth >> 0
        var offset = oceanWidth / 2
        vertices.push(Qt.vector3d((-offset + x) * gridSize, y * gridSize, z * gridSize))
      }
    }
  }

}
