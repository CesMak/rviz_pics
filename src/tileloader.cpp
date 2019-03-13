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

static size_t replaceRegex(const boost::regex &ex, std::string &str,
                           const std::string &replace) {
  std::string::const_iterator start = str.begin(), end = str.end();
  boost::match_results<std::string::const_iterator> what;
  boost::match_flag_type flags = boost::match_default;
  size_t count = 0;
  while (boost::regex_search(start, end, what, ex, flags)) {
    str.replace(what.position(), what.length(), replace);
    start = what[0].second;
    count++;
  }
  return count;
}

void TileLoader::MapTile::abortLoading() {
  if (reply_) {
    reply_->abort();
    reply_ = nullptr;
  }
}

bool TileLoader::MapTile::hasImage() const { return !image_.isNull(); }


TileLoader::TileLoader(QObject *parent)
    : QObject(parent) {
 const std::string package_path = ros::package::getPath("rviz_pics");
  if (package_path.empty()) {
    throw std::runtime_error("package 'rviz_pics' not found to create storage folder");
  }

    cache_path_ = QDir::cleanPath(QString::fromStdString(package_path));

latitude_ = 0.0;
longitude_ = 0.0;
zoom_ = 18;
blocks_ = 1;
center_tile_x_ = 0;
center_tile_y_ = 0;
origin_offset_x_ = 0;
origin_offset_y_ = 0;
      }

bool TileLoader::insideCentreTile(double lat, double lon) const {
  double x, y;
  latLonToTileCoords(lat, lon, zoom_, x, y);
  return (std::floor(x) == center_tile_x_ && std::floor(y) == center_tile_y_);
}

void TileLoader::start() {
  //  discard previous set of tiles and all pending requests
  abort();

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


  checkIfLoadingComplete();
}

double TileLoader::resolution() const {
  return zoomToResolution(latitude_, zoom_);
}

/// @see http://wiki.openstreetmap.org/wiki/Slippy_map_tilenames
/// For explanation of these calculations.
void TileLoader::latLonToTileCoords(double lat, double lon, unsigned int zoom,
                                    double &x, double &y) {
  if (zoom > 31) {
    throw std::invalid_argument("Zoom level " + std::to_string(zoom) +
                                " too high");
  } else if (lat < -85.0511 || lat > 85.0511) {
    throw std::invalid_argument("Latitude " + std::to_string(lat) + " invalid");
  } else if (lon < -180 || lon > 180) {
    throw std::invalid_argument("Longitude " + std::to_string(lon) +
                                " invalid");
  }

  const double rho = M_PI / 180;
  const double lat_rad = lat * rho;

  unsigned int n = (1 << zoom);
  x = n * ((lon + 180) / 360.0);
  y = n * (1 - (std::log(std::tan(lat_rad) + 1 / std::cos(lat_rad)) / M_PI)) /
      2;
  ROS_DEBUG_STREAM( "Center tile coords: " << x << ", " << y );
}

double TileLoader::zoomToResolution(double lat, unsigned int zoom) {
  const double lat_rad = lat * M_PI / 180;
  return 156543.034 * std::cos(lat_rad) / (1 << zoom);
}

bool TileLoader::checkIfLoadingComplete() {
  const bool loaded =
      std::all_of(my_tiles_.begin(), my_tiles_.end(),
                  [](const MapTile &tile) { return tile.hasImage(); });
  if (loaded) {
    emit finishedLoading();
  }
  return loaded;
}

void TileLoader::abort() {
  my_tiles_.clear();
}
