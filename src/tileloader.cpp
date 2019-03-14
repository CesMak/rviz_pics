/*
 * TileLoader.cpp
 *
 *  Copyright (c) 2019 Markus Lamprecht BSD license.
 *
 *  This file is part of rviz_pics
 *
 *	Created on: 13/03/2019
 */

#include "tileloader.h"

#include <QDir>
#include <QFile>
#include <QImage>
#include <QImageReader>
#include <QNetworkProxy>
#include <QNetworkRequest>
#include <QStringList>
#include <QUrl>
#include <QVariant>
#include <boost/regex.hpp>
#include <functional> // for std::hash
#include <ros/package.h>
#include <ros/ros.h>
#include <stdexcept>

TileLoader::TileLoader(QObject *parent) : QObject(parent) {

}

void TileLoader::start() {
  //  discard previous set of tiles and all pending requests
  abort();

  const std::string package_path = ros::package::getPath("rviz_pics");
  QString my_path = QDir::cleanPath(QString::fromStdString(package_path) +
                                    QDir::separator() + QString("map"));

  QDir directory(my_path);
  QStringList image_names = directory.entryList(QStringList() << "*.jpg"
                                                              << "*.JPG",
                                                QDir::Files);

  std::cout << "Number of Images to add: " << image_names.length() << std::endl;

  // /home/markus/ros_space/alfons_ws/src/alfons/rviz_pics/storage_pics/5484377250584657929/x0_y0_z0_0_0_0_0.jpg
  // TODO: 
  for (int i = 0; i < image_names.length(); i++) {
    const QString full_path = my_path + QDir::separator() + image_names[i];
    QStringList list = image_names[i].split('_');

    for(int j=0;j<list.length();j++)
    {
      std::cout<<list[j].toUtf8().constData()<<std::endl;
    }

    double x = list[0].toDouble();
    double y = list[1].toDouble();
    double z = list[2].toDouble();
    double xq = list[3].toDouble();
    double yq = list[4].toDouble();
    double zq = list[5].toDouble();
    double wq = list[6].toDouble();
    std::cout << x << " " << y << " " << z << "      " << xq << " " << yq << " "
              << zq << " " << wq << std::endl;
    QFile tile(full_path);
    if (tile.exists()) {
      QImage image(full_path);
      my_tiles_.push_back(
          MapTile(x, y, z, xq, yq, zq, wq,
                  image)); // The tiles that are pushed back in here are loaded!
      std::cout << "Img found at: " << full_path.toUtf8().constData()
                << " was added" << std::endl;
    } else {
      std::cout << "Img NOT found at: " << full_path.toUtf8().constData()
                << " not added" << std::endl;
    }
  }

  checkIfLoadingComplete();
}

double TileLoader::resolution() const { return 0.4; }

bool TileLoader::MapTile::hasImage() const { return !image_.isNull(); }

bool TileLoader::checkIfLoadingComplete() {
  const bool loaded =
      std::all_of(my_tiles_.begin(), my_tiles_.end(),
                  [](const MapTile &tile) { return tile.hasImage(); });
  if (loaded) {
    emit finishedLoading();
  }
  return loaded;
}

void TileLoader::abort() { my_tiles_.clear(); }
