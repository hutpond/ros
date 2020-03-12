/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QOpenDriveObject.h
 * Author: liuzheng
 * Date: 2019/7/16
 * Description: OpenDrive解析、保存
********************************************************/
#include <QXmlStreamReader>
#include <QXmlStreamWriter>
#include <QFile>
#include <QDateTime>
#include <QSharedPointer>
#include "QOpenDriveObject.h"
#include "GlobalDefine.h"
//#include "QProjectObject.h"
//#include "gps.h"

QOpenDriveObject::QOpenDriveObject(QObject *parent) : QObject(parent)
{

}

void QOpenDriveObject::readOpenDriveFile(const QString &fileName,
                                         QList<QSharedPointer<MapPoint>> &reference,
                                         QList<QSharedPointer<MapPoint>> &road_side)
{
  reference.clear();
  QFile file(fileName);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    return;

  QXmlStreamReader reader(&file);
  int type = 0;
  while (!reader.atEnd()) {
    QXmlStreamReader::TokenType nType = reader.readNext();
    switch (nType) {
      case QXmlStreamReader::StartDocument:
        break;
      case QXmlStreamReader::Comment:
        break;
      case QXmlStreamReader::ProcessingInstruction:
        break;
      case QXmlStreamReader::DTD:
          break;
      case QXmlStreamReader::StartElement: {
          QString strElementName = reader.name().toString();

          // road
          if (QString::compare(strElementName, "road") == 0) {
          }
          // section
          if (QString::compare(strElementName, "section") == 0) {
          }
          // lane
          if (QString::compare(strElementName, "lane") == 0) {
          }
          // curve
          if (QString::compare(strElementName, "curve") == 0) {
            QXmlStreamAttributes attributes = reader.attributes();
            type = attributes.value("id").toInt();
          }
          // point
          if (QString::compare(strElementName, "point") == 0) {
            QXmlStreamAttributes attributes = reader.attributes();
            QSharedPointer<MapPoint> data(new MapPoint);

            data->east = attributes.value("east").toDouble();
            data->north = attributes.value("north").toDouble();
            data->up = attributes.value("up").toDouble();
            data->lat = attributes.value("lat").toDouble();
            data->lon = attributes.value("lon").toDouble();
            data->alt =  attributes.value("alt").toDouble();
            data->pitch = attributes.value("pitch").toDouble();
            data->roll = attributes.value("roll").toDouble();
            data->yaw = attributes.value("yaw").toDouble();
            data->bush = attributes.value("bush").toInt();
            if (data->yaw < 0) {
              data->yaw += 360;
            }
            if (type == 0) {
              reference.push_back(data);
            }
            else {
              road_side.push_back(data);
            }
          }
          break;
        }
      case QXmlStreamReader::EndDocument:
        break;
      default:
        break;
    }
  }
  if (reader.hasError()) {
    ;
  }

  file.close();
}

void QOpenDriveObject::writeOpenDriveFile(const QString &fileName,
                                          const QList<QSharedPointer<MapPoint>> &reference,
                                          const QList<QSharedPointer<MapPoint>> &road_side)
{
  QFile file(fileName);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    return;
  QXmlStreamWriter stream(&file);
  stream.setAutoFormatting(true);

  stream.writeStartDocument();
  stream.writeStartElement("OpenDRIVE");

  stream.writeStartElement("header");           // <-- header
  stream.writeAttribute("revMajor", "1");
  stream.writeAttribute("revMinor", "1");
//  stream.writeAttribute("name", name);
  stream.writeAttribute("version", "1.0");
  stream.writeAttribute("date", QDateTime::currentDateTime().toString("ddd MMMM d hh:mm:ss yyyy"));
  stream.writeAttribute("north", QString("%1").arg(0.0, -1, 'e', 16, QLatin1Char('0')));
  stream.writeAttribute("south", QString("%1").arg(0.0, -1, 'e', 16, QLatin1Char('0')));
  stream.writeAttribute("east", QString("%1").arg(0.0, -1, 'e', 16, QLatin1Char('0')));
  stream.writeAttribute("west", QString("%1").arg(0.0, -1, 'e', 16, QLatin1Char('0')));
  stream.writeEndElement();                     // header -->

  stream.writeStartElement("road");              // <-- road
  stream.writeAttribute("name", "");
  stream.writeAttribute("length", QString("%1").arg(1.34, -1, 'e', 16, QLatin1Char('0')));
  stream.writeAttribute("id", QString::number(0));
  stream.writeAttribute("junction", QString::number(1));

  stream.writeStartElement("section");      // <-- lane section 1
  stream.writeAttribute("id", QString("0"));

  stream.writeStartElement("lane");           // <-- lane 0
  stream.writeAttribute("id", QString("0"));

  stream.writeStartElement("curve");           // <-- curve center
  stream.writeAttribute("id", QString("0"));
  for (const auto &point : reference) {
    stream.writeStartElement("point");          // <-- point
    stream.writeAttribute("east", QString("%1").arg(point->east, -1, 'e', 16, QLatin1Char('0')));
    stream.writeAttribute("north", QString("%1").arg(point->north, -1, 'e', 16, QLatin1Char('0')));
    stream.writeAttribute("up", QString("%1").arg(point->up, -1, 'e', 16, QLatin1Char('0')));
    stream.writeAttribute("lon", QString("%1").arg(point->lon, -1, 'e', 16, QLatin1Char('0')));
    stream.writeAttribute("lat", QString("%1").arg(point->lat, -1, 'e', 16, QLatin1Char('0')));
    stream.writeAttribute("alt", QString("%1").arg(point->alt, -1, 'e', 16, QLatin1Char('0')));
    stream.writeAttribute("pitch", QString("%1").arg(point->pitch, -1, 'e', 16, QLatin1Char('0')));
    stream.writeAttribute("roll", QString("%1").arg(point->roll, -1, 'e', 16, QLatin1Char('0')));
    stream.writeAttribute("yaw", QString("%1").arg(point->yaw, -1, 'e', 16, QLatin1Char('0')));
    stream.writeAttribute("bush", QString::number(point->bush));
    stream.writeEndElement();  // point -->
  }
  stream.writeEndElement();  // curve center -->

  stream.writeStartElement("curve");              // <-- curve left
  stream.writeAttribute("id", QString("-1"));
  for (const auto &point : road_side) {
    stream.writeStartElement("point");             // <-- point
    stream.writeAttribute("east", QString("%1").arg(point->east, -1, 'e', 16, QLatin1Char('0')));
    stream.writeAttribute("north", QString("%1").arg(point->north, -1, 'e', 16, QLatin1Char('0')));
    stream.writeAttribute("up", QString("%1").arg(point->up, -1, 'e', 16, QLatin1Char('0')));
    stream.writeAttribute("lon", QString("%1").arg(point->lon, -1, 'e', 16, QLatin1Char('0')));
    stream.writeAttribute("lat", QString("%1").arg(point->lat, -1, 'e', 16, QLatin1Char('0')));
    stream.writeAttribute("alt", QString("%1").arg(point->alt, -1, 'e', 16, QLatin1Char('0')));
    stream.writeEndElement();    // point -->
  }
  stream.writeEndElement();    // curve left -->

  stream.writeEndElement();  // lane 0 -->
  stream.writeEndElement();    // lane section 1 -->
  stream.writeEndElement();    // road -->
  stream.writeEndElement();    // OpenDRIVE -->
  stream.writeEndDocument();

  file.close();
}
