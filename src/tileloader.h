/*
 * TileLoader.h
 *
 *  Copyright (c) 2014 Gaeth Cross. Apache 2 License.
 *
 *  This file is part of rviz_satellite.
 *
 *	Created on: 07/09/2014
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
    // MapTile(int x, int y, int z, QNetworkReply *reply = nullptr)
    //     : x_(x), y_(y), z_(z), reply_(reply) {}
      
    // MapTile(int x, int y, int z, QImage & image)
    //   : x_(x), y_(y), z_(z), reply_(nullptr), image_(image) {}


    MapTile(double x, double y, double z, double qx, double qy, double qz, double qw, QImage & image)
      : pos_x_(x), pos_y_(y), pos_z_(z), qx_(qx), qy_(qy), qz_(qz), qw_(qw), image_(image) {}

    /// X tile coordinate.
    int posX() const { return pos_x_; }

    /// Y tile coordinate.
    int posY() const { return pos_y_; }
      
    /// Z tile zoom value.
    int posZ() const { return pos_z_; }

    /// X tile coordinate.
    double qx() const { return qx_; }

    /// Y tile coordinate.
    double qy() const { return qy_; }
      
    /// Z tile zoom value.
    double qz() const { return qz_; }

    /// Z tile zoom value.
    double qw() const { return qz_; }

    /// Abort the network request for this tile, if applicable.
    void abortLoading();

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

  /// X index of central tile.
  int centerTileX() const { return center_tile_x_; }

  /// Y index of central tile.
  int centerTileY() const { return center_tile_y_; }

  /// Fraction of a tile to offset the origin (X).
  double originOffsetX() const { return origin_offset_x_; }

  /// Fraction of a tile to offset the origin (Y).
  double originOffsetY() const { return origin_offset_y_; }

  /// Test if (lat,lon) falls inside centre tile.
  bool insideCentreTile(double lat, double lon) const;

  /// Convert lat/lon to a tile index with mercator projection.
  static void latLonToTileCoords(double lat, double lon, unsigned int zoom,
                                 double &x, double &y);

  /// Convert latitude and zoom level to ground resolution.
  static double zoomToResolution(double lat, unsigned int zoom);

  /// Current set of tiles.
  const std::vector<MapTile> &tiles() const { return tiles_; }
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

  void finishedRequest(QNetworkReply *reply);

private:

  /// Check if loading is complete. Emit signal if appropriate.
  bool checkIfLoadingComplete();

  /// URI for tile [x,y]
  QUrl uriForTile(int x, int y) const;

  /// Get name for cached tile [x,y,z]
  QString cachedNameForTile(int x, int y, int z) const;

  /// Get file path for cached tile [x,y,z].
  QString cachedPathForTile(int x, int y, int z) const;

  /// Maximum number of tiles for the zoom level
  int maxTiles() const;

  unsigned int zoom_;
  int blocks_;
  int center_tile_x_;
  int center_tile_y_;
  double origin_offset_x_;
  double origin_offset_y_;

  std::shared_ptr<QNetworkAccessManager> qnam_;
  QString cache_path_;

  std::string object_uri_;

  std::vector<MapTile> tiles_;
  std::vector<MapTile> my_tiles_;
};

#endif // TILELOADER_H
