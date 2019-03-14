/*
 * TileLoader.h
 *
 *  Copyright (c) 2019 Markus Lamprecht BSD license.
 *
 *  This file is part of rviz_pics
 *
 *	Created on: 13/03/2019
 */

#ifndef TILELOADER_H
#define TILELOADER_H

#include <QObject>
#include <QImage>
#include <QNetworkAccessManager>
#include <QString>
#include <QNetworkReply>
#include <vector>
#include <memory>

class TileLoader : public QObject {
  Q_OBJECT
public:
  class MapTile {
  public:
    MapTile(double x, double y, double z, double qx, double qy, double qz, double qw, QImage & image)
      : pos_x_(x), pos_y_(y), pos_z_(z), qx_(qx), qy_(qy), qz_(qz), qw_(qw), image_(image) {}

    /// X position of tile
    int posX() const { return pos_x_; }

    /// Y 
    int posY() const { return pos_y_; }
      
    /// Z 
    int posZ() const { return pos_z_; }

    /// quaternion x 
    double qx() const { return qx_; }

    /// quaternion
    double qy() const { return qy_; }
      
    /// quaternion
    double qz() const { return qz_; }

    /// quaternion
    double qw() const { return qw_; }

    /// Has a tile successfully loaded?
    bool hasImage() const;

    /// Image associated with this tile.
    const QImage &image() const { return image_; }
    void setImage(const QImage &image) { image_ = image; }

  private:
    double pos_x_;
    double pos_y_;
    double pos_z_;
    double qx_;
    double qy_;
    double qz_;
    double qw_;
    QImage image_;
  };

explicit TileLoader(QObject *parent = nullptr);

  /// Start loading tiles asynchronously.
  void start();
  void addMyImage();

  /// Meters/pixel of the tiles.
  double resolution() const;

  const std::vector<MapTile> &Mytiles() const { return my_tiles_; }

  /// Cancel all current requests.
  void abort();

signals:

  void initiatedRequest(QNetworkRequest request);

  void receivedImage(QNetworkRequest request);

  void finishedLoading();

  void errorOcurred(QString description);

public slots:

private slots:

private:

  /// Check if loading is complete. Emit signal if appropriate.
  bool checkIfLoadingComplete();

  std::vector<MapTile> my_tiles_;
};

#endif // TILELOADER_H
