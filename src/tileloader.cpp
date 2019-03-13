/*
 * TileLoader.cpp
 *
 *  Copyright (c) 2014 Gaeth Cross. Apache 2 License.
 *
 *  This file is part of rviz_satellite.
 *
 *	Created on: 07/09/2014
 */

#include "tileloader.h"

#include <QUrl>
#include <QNetworkRequest>
#include <QNetworkProxy>
#include <QVariant>
#include <QDir>
#include <QStringList>
#include <QFile>
#include <QImage>
#include <QImageReader>
#include <stdexcept>
#include <boost/regex.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <functional> // for std::hash

// static size_t replaceRegex(const boost::regex &ex, std::string &str,
//                            const std::string &replace) {
//   std::string::const_iterator start = str.begin(), end = str.end();
//   boost::match_results<std::string::const_iterator> what;
//   boost::match_flag_type flags = boost::match_default;
//   size_t count = 0;
//   while (boost::regex_search(start, end, what, ex, flags)) {
//     str.replace(what.position(), what.length(), replace);
//     start = what[0].second;
//     count++;
//   }
//   return count;
// }

void TileLoader::MapTile::abortLoading() {
  // if (reply_) {
  //   reply_->abort();
  //   reply_ = nullptr;
  // }
}

bool TileLoader::MapTile::hasImage() const { return !image_.isNull(); }

TileLoader::TileLoader(QObject *parent)
    : QObject(parent){
  assert(blocks_ >= 0);

  const std::string package_path = ros::package::getPath("rviz_pics");
  if (package_path.empty()) {
    throw std::runtime_error("package 'rviz_pics' not found to create storage folder");
  }

  std::hash<std::string> hash_fn;
  cache_path_ =
      QDir::cleanPath(QString::fromStdString(package_path) + QDir::separator() +
                      QString("storage_pics") + QDir::separator() +
                      QString::number(hash_fn(object_uri_)));

  QDir dir(cache_path_);
  if (!dir.exists() && !dir.mkpath(".")) {
    throw std::runtime_error("Failed to create cache folder: " +
                             cache_path_.toStdString());
  }


  std::cout<<"path is:  "<<package_path<<std::endl;

  /// @todo: some kind of error checking of the URL

  //  calculate center tile coordinates
  // double x, y;
  // latLonToTileCoords(latitude_, longitude_, zoom_, x, y);
  // center_tile_x_ = std::floor(x);
  // center_tile_y_ = std::floor(y);
  // //  fractional component
  // origin_offset_x_ = x - center_tile_x_;
  // origin_offset_y_ = y - center_tile_y_;
}

bool TileLoader::insideCentreTile(double lat, double lon) const {
  // double x, y;
  // latLonToTileCoords(lat, lon, zoom_, x, y);
  // return (std::floor(x) == center_tile_x_ && std::floor(y) == center_tile_y_);
  return true;
}

void TileLoader::addMyImage()
{
  const std::string package_path = ros::package::getPath("rviz_pics");
  QString my_path = QDir::cleanPath(QString::fromStdString(package_path) + QDir::separator() +
                      QString("map"));

  QDir directory(my_path);
  QStringList image_names = directory.entryList(QStringList() << "*.jpg" << "*.JPG", QDir::Files);

  std::cout<<"Number of Images to add: "<<image_names.length()<<std::endl;

// /home/markus/ros_space/alfons_ws/src/alfons/rviz_pics/storage_pics/5484377250584657929/x0_y0_z0_0_0_0_0.jpg
for(int i = 0;i<image_names.length();i++)
{
  const QString full_path  =my_path+QDir::separator() +image_names[i];
  QStringList list = image_names[i].split('_'); 
  double x = list[0].toDouble();
  double y = list[1].toDouble();
  double z = list[2].toDouble();
  double xq = list[3].toDouble();
  double yq = list[4].toDouble();
  double zq = list[5].toDouble();
  double wq = list[6].toDouble();
  std::cout<<x<<" "<<y<<" "<<z<<"      "<<xq<<" "<<yq<<" "<<zq<<" "<<wq<<std::endl;
  QFile tile(full_path);
    if (tile.exists()) {
      QImage image(full_path);
      my_tiles_.push_back(MapTile(x, y, z, xq, yq, zq, wq, image)); // The tiles that are pushed back in here are loaded!
      std::cout<<"Img found at: "<<full_path.toUtf8().constData()<< " was added" <<std::endl;
    } 
    else {
      std::cout<<"Img NOT found at: "<<full_path.toUtf8().constData()<<" not added"<<std::endl;
    } 
}
}


void TileLoader::start() {
  //  discard previous set of tiles and all pending requests
 // abort();
 addMyImage(); // markus Add!

  // ROS_INFO("loading %d blocks around tile=(%d,%d)", blocks_, center_tile_x_, center_tile_y_ );

  // qnam_.reset( new QNetworkAccessManager(this) );
  // QObject::connect(qnam_.get(), SIGNAL(finished(QNetworkReply *)), this,
  //                  SLOT(finishedRequest(QNetworkReply *)));
  // qnam_->proxyFactory()->setUseSystemConfiguration ( true );

  // //  determine what range of tiles we can load
  // const int min_x = std::max(0, center_tile_x_ - blocks_);
  // const int min_y = std::max(0, center_tile_y_ - blocks_);
  // const int max_x = std::min(maxTiles(), center_tile_x_ + blocks_);
  // const int max_y = std::min(maxTiles(), center_tile_y_ + blocks_);

  // //  initiate requests
  // for (int y = min_y; y <= max_y; y++) {
  //   for (int x = min_x; x <= max_x; x++) {
  //     // Generate filename
  //     const QString full_path = cachedPathForTile(x, y, zoom_);

  //     // Check if tile is already in the cache
  //     QFile tile(full_path);
  //     if (tile.exists()) {
  //       QImage image(full_path);
  //       //tiles_.push_back(MapTile(x, y, zoom_, image)); // THe tiles that are pushed back in here are loaded!
  //        std::cout<<"The image exists offline at: "<<full_path.toUtf8().constData()<<std::endl;
  //     } else {
  //       const QUrl uri = uriForTile(x, y);
  //       //  send request
  //       QNetworkRequest request = QNetworkRequest(uri);
  //       auto const userAgent = QByteArray("rviz_satellite/0.0.2 (+https://github.com/gareth-cross/rviz_satellite)");
  //       request.setRawHeader(QByteArray("User-Agent"), userAgent);
  //       QNetworkReply *rep = qnam_->get(request);
  //       emit initiatedRequest(request);
  //      // tiles_.push_back(MapTile(x, y, zoom_, rep));
  //     }
  //   }
  // }

  // checkIfLoadingComplete();
}

double TileLoader::resolution() const {
  return 1.0;
}

/// @see http://wiki.openstreetmap.org/wiki/Slippy_map_tilenames
/// For explanation of these calculations.
void TileLoader::latLonToTileCoords(double lat, double lon, unsigned int zoom,
                                    double &x, double &y) {
  // if (zoom > 31) {
  //   throw std::invalid_argument("Zoom level " + std::to_string(zoom) +
  //                               " too high");
  // } else if (lat < -85.0511 || lat > 85.0511) {
  //   throw std::invalid_argument("Latitude " + std::to_string(lat) + " invalid");
  // } else if (lon < -180 || lon > 180) {
  //   throw std::invalid_argument("Longitude " + std::to_string(lon) +
  //                               " invalid");
  // }

  // const double rho = M_PI / 180;
  // const double lat_rad = lat * rho;

  // unsigned int n = (1 << zoom);
  // x = n * ((lon + 180) / 360.0);
  // y = n * (1 - (std::log(std::tan(lat_rad) + 1 / std::cos(lat_rad)) / M_PI)) /
  //     2;
  // ROS_DEBUG_STREAM( "Center tile coords: " << x << ", " << y );
}

double TileLoader::zoomToResolution(double lat, unsigned int zoom) {
 // const double lat_rad = lat * M_PI / 180;
  return 0.0;
}

void TileLoader::finishedRequest(QNetworkReply *reply) {
  // const QNetworkRequest request = reply->request();

  // //  find corresponding tile
  // const std::vector<MapTile>::iterator it =
  //     std::find_if(tiles_.begin(), tiles_.end(),
  //                  [&](const MapTile &tile) { return tile.reply() == reply; });
  // if (it == tiles_.end()) {
  //   //  removed from list already, ignore this reply
  //   return;
  // }
  // MapTile &tile = *it;

  // if (reply->error() == QNetworkReply::NoError) {
  //   //  decode an image
  //   QImageReader reader(reply);
  //   if (reader.canRead()) {
  //     QImage image = reader.read();
  //     tile.setImage(image); // wenn auskommentiert mach keinen unterschied!
  //     image.save(cachedPathForTile(tile.x(), tile.y(), tile.z()), "JPEG");
  //     emit receivedImage(request);
  //   } else {
  //     //  probably not an image
  //     QString err;
  //     err = "Unable to decode image at " + request.url().toString();
  //     emit errorOcurred(err);
  //   }
  // } else {
  //   const QString err = "Failed loading " + request.url().toString() +
  //                       " with code " + QString::number(reply->error());
  //   emit errorOcurred(err);
  // }

  // checkIfLoadingComplete();
}

bool TileLoader::checkIfLoadingComplete() {
  // const bool loaded =
  //     std::all_of(tiles_.begin(), tiles_.end(),
  //                 [](const MapTile &tile) { return tile.hasImage(); });
  // if (loaded) {
  //   emit finishedLoading();
  // }
  return true;
}

// QUrl TileLoader::uriForTile(int x, int y) const {
//   std::string object = object_uri_;
//   //  place {x},{y},{z} with appropriate values
//   replaceRegex(boost::regex("\\{x\\}", boost::regex::icase), object,
//                std::to_string(x));
//   replaceRegex(boost::regex("\\{y\\}", boost::regex::icase), object,
//                std::to_string(y));
//   replaceRegex(boost::regex("\\{z\\}", boost::regex::icase), object,
//                std::to_string(zoom_));

//   const QString qstr = QString::fromStdString(object);
//   return QUrl(qstr);
// }

// QString TileLoader::cachedNameForTile(int x, int y, int z) const {
//   return "x" + QString::number(x) + "_y" + QString::number(y) + "_z" +
//          QString::number(z) + ".jpg";
// }

// QString TileLoader::cachedPathForTile(int x, int y, int z) const {
//   return QDir::cleanPath(cache_path_ + QDir::separator() +
//                          cachedNameForTile(x, y, z));
// }

// int TileLoader::maxTiles() const { return (1 << zoom_) - 1; }

void TileLoader::abort() {
  tiles_.clear();
  //  destroy network access manager
  //qnam_.reset();
}
