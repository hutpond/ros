import QtQuick 2.0

Item {
  Rectangle {
    anchors.fill: parent
    border.width: 1
    border.color: "black"
  }

  Item {
    width: parent.width
    height: parent.height * 0.8
    anchors.centerIn: parent

    property double itemWidth: height * 0.8
    property double spacing: (width - itemWidth * 6) / 7

    Image {
      x: parent.spacing
      width: parent.itemWidth
      height: parent.itemWidth
      anchors.verticalCenter: parent.verticalCenter

      source: "qrc:/image/garbage.png"
    }

    Image {
      x: parent.spacing
      width: parent.itemWidth
      height: parent.itemWidth
      anchors.verticalCenter: parent.verticalCenter

      source: "qrc:/image/garbage.png"
    }

    Image {
      x: parent.spacing
      width: parent.itemWidth
      height: parent.itemWidth
      anchors.verticalCenter: parent.verticalCenter

      source: "qrc:/image/garbage.png"
    }

    Image {
      x: parent.spacing
      width: parent.itemWidth
      height: parent.itemWidth
      anchors.verticalCenter: parent.verticalCenter

      source: "qrc:/image/garbage.png"
    }

    Image {
      x: parent.spacing
      width: parent.itemWidth
      height: parent.itemWidth
      anchors.verticalCenter: parent.verticalCenter

      source: "qrc:/image/garbage.png"
    }

    Image {
      x: parent.spacing
      width: parent.itemWidth
      height: parent.itemWidth
      anchors.verticalCenter: parent.verticalCenter

      source: "qrc:/image/garbage.png"
    }
  }

}
