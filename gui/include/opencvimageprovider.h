#ifndef OPENCVIMAGEPROVIDER_H
#define OPENCVIMAGEPROVIDER_H


#include <QObject>
#include <QImage>
#include <QQuickImageProvider>
#include <QtQml>

class OpencvImageProvider : public QObject, public QQuickImageProvider
{
    Q_OBJECT
    QML_ELEMENT
public:
    OpencvImageProvider(QObject *parent = nullptr);

    QImage requestImage(const QString &id, QSize *size, const QSize &requestedSize) override;

public slots:
    void updateImage(const QImage &image);

signals:
    void imageChanged();

private:
    QImage image;
};

#endif // OPENCVIMAGEPROVIDER_H
