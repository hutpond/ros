#ifndef QREADDATAROSOBJECT_H
#define QREADDATAROSOBJECT_H

#include <QObject>

class QReadDataRosObject : public QObject
{
  Q_OBJECT
public:
  explicit QReadDataRosObject(QObject *parent = nullptr);

signals:

public slots:
};

#endif // QREADDATAROSOBJECT_H
