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
#include "QOpenDriveObject.h"
#include "QProjectObject.h"

QOpenDriveObject::QOpenDriveObject(QObject *parent) : QObject(parent)
{

}

void QOpenDriveObject::readOpenDriveFile(const QString &path, const QString &name,
                                         QList<QSharedPointer<MapBinData>> &datas)
{
  QString fileName = path + name + ".xord";
  QFile file(fileName);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    return;
  QXmlStreamReader reader(&file);
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
          if (QString::compare(strElementName, "geometry") == 0) {
            QSharedPointer<MapBinData> data(new MapBinData);
            QXmlStreamAttributes attributes = reader.attributes();
            data->x = attributes.value("x").toDouble();
            data->y = attributes.value("y").toDouble();
            data->z = attributes.value("height").toDouble();
            data->pitch = attributes.value("pitch").toDouble();
            data->roll = attributes.value("roll").toDouble();
            data->yaw = attributes.value("yaw").toDouble();
            data->lat = attributes.value("lat").toDouble();
            data->lon = attributes.value("lon").toDouble();
            data->alt = attributes.value("alt").toDouble();

            datas.append(data);
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

void QOpenDriveObject::writeOpenDriveFile(const QString &path, const QString &name,
                                          const QList<QSharedPointer<MapBinData>> &datas)
{
  QString fileName = path + name + ".xord";
  QFile file(fileName);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    return;
  QXmlStreamWriter stream(&file);
  stream.setAutoFormatting(true);

  stream.writeStartDocument();
  stream.writeStartElement("OpenDRIVE");

  stream.writeStartElement("header");
  stream.writeAttribute("revMajor", "1");
  stream.writeAttribute("revMinor", "1");
  stream.writeAttribute("name", name);
  stream.writeAttribute("version", "1.0");
  stream.writeAttribute("date", QDateTime::currentDateTime().toString("ddd MMMM d hh:mm:ss yyyy"));
  stream.writeAttribute("north", QString("%1").arg(0.0, -1, 'e', 16, QLatin1Char('0')));
  stream.writeAttribute("south", QString("%1").arg(0.0, -1, 'e', 16, QLatin1Char('0')));
  stream.writeAttribute("east", QString("%1").arg(0.0, -1, 'e', 16, QLatin1Char('0')));
  stream.writeAttribute("west", QString("%1").arg(0.0, -1, 'e', 16, QLatin1Char('0')));
  stream.writeEndElement(); // header

  stream.writeStartElement("road");
  stream.writeAttribute("name", "");
  stream.writeAttribute("length", QString("%1").arg(1.34, -1, 'e', 16, QLatin1Char('0')));
  stream.writeAttribute("id", QString::number(28));
  stream.writeAttribute("junction", QString::number(1));
  stream.writeStartElement("road");

  stream.writeStartElement("planView");
  auto it = datas.begin();
  MapBinData prev;
  if (it != datas.end()) {
    const auto &data = *it;
    prev = *data;
  }
  double sSum = 0;
  for (; it != datas.end(); ++it) {
    const auto &data = *it;
    double length = data->distance(prev);
    sSum += length;
    stream.writeStartElement("geometry");
    stream.writeAttribute("s", QString("%1").arg(sSum, -1, 'e', 16, QLatin1Char('0')));
    stream.writeAttribute("x", QString("%1").arg(data->x, -1, 'e', 16, QLatin1Char('0')));
    stream.writeAttribute("y", QString("%1").arg(data->y, -1, 'e', 16, QLatin1Char('0')));
    stream.writeAttribute("hdg", QString("%1").arg(data->angle(), -1, 'e', 16, QLatin1Char('0')));
    stream.writeAttribute("length", QString("%1").arg(length, -1, 'e', 16, QLatin1Char('0')));
    stream.writeAttribute("lat", QString("%1").arg(data->lat, -1, 'e', 16, QLatin1Char('0')));
    stream.writeAttribute("lon", QString("%1").arg(data->lon, -1, 'e', 16, QLatin1Char('0')));
    stream.writeAttribute("alt", QString("%1").arg(data->alt, -1, 'e', 16, QLatin1Char('0')));
    stream.writeAttribute("pitch", QString("%1").arg(data->pitch, -1, 'e', 16, QLatin1Char('0')));
    stream.writeAttribute("roll", QString("%1").arg(data->roll, -1, 'e', 16, QLatin1Char('0')));
    stream.writeAttribute("yaw", QString("%1").arg(data->yaw, -1, 'e', 16, QLatin1Char('0')));
    stream.writeAttribute("height", QString("%1").arg(data->z, -1, 'e', 16, QLatin1Char('0')));
    stream.writeStartElement("line");
    stream.writeEndElement(); // line
    stream.writeEndElement(); // geometry
  }
  stream.writeEndElement(); // planView

  stream.writeEndElement(); // road
  stream.writeEndElement(); // OpenDRIVE
  stream.writeEndDocument();

  file.close();
}

