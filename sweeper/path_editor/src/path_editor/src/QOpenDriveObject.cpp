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
                                         QList<QSharedPointer<Point>> &points)
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
            QSharedPointer<Point> point(new Point);
            QXmlStreamAttributes attributes = reader.attributes();
            if (attributes.hasAttribute("x") && attributes.hasAttribute("y")) {
              point->x = attributes.value("x").toDouble();
              point->y = attributes.value("y").toDouble();
              points.append(point);
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

void QOpenDriveObject::writeOpenDriveFile(const QString &path, const QString &name,
                                          const QList<QSharedPointer<Point>> &points)
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
  auto it = points.begin();
  Point prev;
  if (it != points.end()) {
    const auto &point = *it;
    prev = *point;
  }
  double sSum = 0;
  for (; it != points.end(); ++it) {
    const auto &point = *it;
    double length = point->distance(prev);
    sSum += length;
    stream.writeStartElement("geometry");
    stream.writeAttribute("s", QString("%1").arg(sSum, -1, 'e', 16, QLatin1Char('0')));
    stream.writeAttribute("x", QString("%1").arg(point->x, -1, 'e', 16, QLatin1Char('0')));
    stream.writeAttribute("y", QString("%1").arg(point->y, -1, 'e', 16, QLatin1Char('0')));
    stream.writeAttribute("hdg", QString("%1").arg(point->angle(), -1, 'e', 16, QLatin1Char('0')));
    stream.writeAttribute("length", QString("%1").arg(length, -1, 'e', 16, QLatin1Char('0')));
    stream.writeAttribute("lat", QString("%1").arg(point->lat, -1, 'e', 16, QLatin1Char('0')));
    stream.writeAttribute("lon", QString("%1").arg(point->lon, -1, 'e', 16, QLatin1Char('0')));
    stream.writeAttribute("height", QString("%1").arg(point->height, -1, 'e', 16, QLatin1Char('0')));
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

