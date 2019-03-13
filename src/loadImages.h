/*
 * LoadImages.h
 *
 *  Copyright (c) 2019 Markus Lamprecht BSD license.
 *
 *  This file is part of rviz_pics
 *
 *	Created on: 13/03/2019
 */

#ifndef LOAD_IMAGES_H
#define LOAD_IMAGES_H

// NOTE: workaround for issue: https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/time.h>
#include <rviz/display.h>
#include <sensor_msgs/NavSatFix.h>
#include <rviz_pics/rviz_scale.h> // scaling message

#include <OGRE/OgreTexture.h>
#include <OGRE/OgreMaterial.h>
#endif  //  Q_MOC_RUN

#include <QObject>
#include <QtConcurrentRun>
#include <QFuture>
#include <QByteArray>
#include <QFile>
#include <QNetworkRequest>

#include <memory>
#include <tileloader.h>

namespace Ogre {
class ManualObject;
}

namespace rviz {

class FloatProperty;
class IntProperty;
class Property;
class StringProperty;
class TfFrameProperty;

/**
 * @class LoadImages
 * @brief Displays a satellite map along the XY plane.
 */
class LoadImages : public Display {
  Q_OBJECT
public:
  LoadImages();
  ~LoadImages() override;

  // Overrides from Display
  void reset() override;
  void update(float, float) override;

protected Q_SLOTS:
  void updateAlpha();
  void updateDrawUnder();

  //Additions:

  //  slots for TileLoader messages
  void receivedImage(QNetworkRequest request);
  void finishedLoading();
  void errorOcurred(QString description);

protected:
  void loadImagery();

  void assembleScene();

  void clear();
  
  void clearGeometry();

  unsigned int map_id_;
  unsigned int scene_id_;

  /// Instance of a tile w/ associated ogre data
  struct MapObject {
    Ogre::ManualObject *object;
    Ogre::TexturePtr texture;
    Ogre::MaterialPtr material;
  };
  std::vector<MapObject> objects_;

  //  properties
  FloatProperty *resolution_property_;
  FloatProperty *alpha_property_;
  Property *draw_under_property_;

  float alpha_;
  bool draw_under_;

  //  tile management
  bool dirty_;
  bool received_msg_;

  std::shared_ptr<TileLoader> loader_;
};

} // namespace rviz

#endif
