import QtQuick 2.0
import QtQuick.Layouts 1.3

Item {

  id: dashBoard
  property int spacing: 20

  Row {
    id: firstRow

    width: parent.width
    height: (parent.height - parent.spacing) / 2
    anchors.top: parent.top
    anchors.left: parent.left

    property double itemWidth: Math.min(width / 5, height)
    spacing: (width - itemWidth * 5) / 4.0

    Image {
      width: parent.itemWidth
      height: parent.itemWidth
      anchors.verticalCenter: parent.verticalCenter

      source: "qrc:/svg/brake.svg"
    }

    Image {
      width: parent.itemWidth
      height: parent.itemWidth
      anchors.verticalCenter: parent.verticalCenter

      source: "qrc:/svg/leftSignalLight.svg"
    }

    Image {
      width: parent.itemWidth
      height: parent.itemWidth
      anchors.verticalCenter: parent.verticalCenter

      source: "qrc:/svg/rightSignalLight.svg"
    }

    Image {
      width: parent.itemWidth
      height: parent.itemWidth
      anchors.verticalCenter: parent.verticalCenter

      source: "qrc:/svg/setting.svg"
    }

    Image {
      width: parent.itemWidth
      height: parent.itemWidth
      anchors.verticalCenter: parent.verticalCenter

      source: "qrc:/svg/water.svg"
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

    Image {
      width: parent.itemWidth
      height: parent.itemWidth
      anchors.verticalCenter: parent.verticalCenter

      source: "qrc:/svg/spout.svg"
    }

    Image {
      width: parent.itemWidth
      height: parent.itemWidth
      anchors.verticalCenter: parent.verticalCenter

      source: "qrc:/svg/pallet.svg"
    }

    Image {
      width: parent.itemWidth
      height: parent.itemWidth
      anchors.verticalCenter: parent.verticalCenter

      source: "qrc:/svg/positionLamp.svg"
    }

    Image {
      width: parent.itemWidth
      height: parent.itemWidth
      anchors.verticalCenter: parent.verticalCenter

      source: "qrc:/svg/highBeam.svg"
    }

    Image {
      width: parent.itemWidth
      height: parent.itemWidth
      anchors.verticalCenter: parent.verticalCenter

      source: "qrc:/svg/lowBeam.svg"
    }
  }
}
