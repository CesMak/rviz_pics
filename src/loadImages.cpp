/*
 * LoadImages.cpp
 *
 *  Copyright (c) 2019 Markus Lamprecht BSD license.
 *
 *  This file is part of rviz_pics
 *
 *	Created on: 13/03/2019
 */

#include <QtGlobal>
#include <QImage>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>  // include for scene_node_
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreImageCodec.h>
#include <OGRE/OgreVector3.h>
#include <QTimer>

#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/grid.h"
#include "rviz/properties/enum_property.h"
#include "rviz/properties/float_property.h" //http://docs.ros.org/jade/api/rviz/html/c++/float__property_8h_source.html
#include "rviz/properties/int_property.h"
#include "rviz/properties/property.h"
#include "rviz/properties/tf_frame_property.h"
#include "rviz/properties/quaternion_property.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/validate_floats.h"
#include "rviz/display_context.h"

#include "loadImages.h"


// this method is required!
// just works for imgs of size 256x256
Ogre::TexturePtr textureFromImage(const QImage &image,
                                  const std::string &name) {
  //  convert to 24bit rgb
  QImage converted = image.convertToFormat(QImage::Format_RGB888).mirrored();
 
  int size_of_stream = (converted.width()*converted.height()*3);

  //  create texture
  Ogre::TexturePtr texture;
  Ogre::DataStreamPtr data_stream;
   data_stream.bind(new Ogre::MemoryDataStream((void *)converted.constBits(),
                                               size_of_stream));
  std::cout<<" data_stream Size: "<<data_stream->size()<<"conv wi: "<<converted.width()<<" conv hi"<< converted.height()<<"  img wi:"<< image.width()<<" img_h: "<<image.height()<<std::endl;
 // Size: 196608conv wi: 256 conv hi256  img wi:256 img_h: 256  -> works
 // Size: 119660conv wi: 257 conv hi155  img wi:257 img_h: 155  -> fails with above mentioned error

  bool stream_size = (data_stream->size()) == (converted.width()*converted.height()*3);  
  std::cout<<stream_size<<" "<<data_stream->size()<<" "<<(converted.width()*converted.height()*3)<<std::endl;

  const Ogre::String res_group =
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME;
  Ogre::TextureManager &texture_manager = Ogre::TextureManager::getSingleton();
  //  swap byte order when going from QImage to Ogre
  texture = texture_manager.loadRawData(name, res_group, data_stream,
                                        converted.width(), converted.height(),
                                        Ogre::PF_B8G8R8, Ogre::TEX_TYPE_2D, 0);


// virtual TexturePtr Ogre::TextureManager::loadRawData 	( 	const String &  	name,
// 		const String &  	group,
// 		DataStreamPtr &  	stream,
// 		ushort  	uWidth,
// 		ushort  	uHeight,
// 		PixelFormat  	format,
// 		TextureType  	texType = TEX_TYPE_2D,
// 		int  	iNumMipmaps = MIP_DEFAULT,
// 		Real  	gamma = 1.0f,
// 		bool  	hwGammaCorrection = false 
// 	) 	
  return texture;
}

namespace rviz {

LoadImages::LoadImages()
    : Display(), map_id_(0), scene_id_(0), dirty_(false),
      received_msg_(false) {

  static unsigned int map_ids = 0;
  map_id_ = map_ids++; //  global counter of map ids

  alpha_property_ = new FloatProperty(
      "Alpha", 0.7, "Amount of transparency to apply to the map.", this,
      SLOT(updateAlpha()));
  alpha_ = alpha_property_->getValue().toFloat();
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);
  alpha_property_->setShouldBeSaved(true);

  draw_under_property_ =
      new Property("Draw Behind", false,
                   "Rendering option, controls whether or not the map is always"
                   " drawn behind everything else.",
                   this, SLOT(updateDrawUnder()));
  draw_under_property_->setShouldBeSaved(true);
  draw_under_ = draw_under_property_->getValue().toBool();

  //  output, resolution of the map in meters/pixel
  resolution_property_ = new FloatProperty(
      "Resolution", 0, "Resolution of the map. Scale the images map meters to pixel between 0 and 1", this, SLOT(updateResolution()));
  resolution_property_->setReadOnly(false);
  resolution_property_->setMin(0);
  resolution_property_->setMax(1);
  resolution_property_->setShouldBeSaved(true);

  reload_property_ =
      new Property("Auto reload", false,
                   "Reload images from folder!",
                   this,SLOT(changeAutoReload()));
  reload_property_->setShouldBeSaved(true);
  auto_reload_ = reload_property_->getValue().toBool();

  // Timer for callback updates:
  timer_ = new QTimer(this);
  connect(timer_, SIGNAL(timeout()), this, SLOT(updateReload()));
  timer_->start(1000); //1sec time specified in ms

    loadImagery();
}

LoadImages::~LoadImages() {
  // unsubscribe();
  clear();
}

void LoadImages::updateReload() {
  if(auto_reload_)
  {
    loader_->start();
    ROS_INFO("reload pressed TODO change to autoreoload enable!");
  }
}

void LoadImages::changeAutoReload() {
  auto_reload_ = reload_property_->getValue().toBool();
}

void LoadImages::updateAlpha() {
  alpha_ = alpha_property_->getFloat();
  dirty_ = true;
  ROS_INFO("Changing alpha to %f", alpha_);
}

void LoadImages::updateResolution() {
  resolution_ = resolution_property_->getFloat();
  dirty_ = true; //  force update
  ROS_INFO("Changing resolution_property_ to %f", resolution_);
}



void LoadImages::updateDrawUnder() {
  /// @todo: figure out why this property only applies to some objects
  draw_under_ = draw_under_property_->getValue().toBool();
  dirty_ = true; //  force update
  ROS_INFO("Changing draw_under to %s", ((draw_under_) ? "true" : "false"));
}

void LoadImages::clear() {
  setStatus(StatusProperty::Warn, "Message", "No map received");
  clearGeometry();
  //  the user has cleared here
  received_msg_ = false;
  //  cancel current imagery, if any
  loader_.reset();
}

void LoadImages::clearGeometry() {
  for (MapObject &obj : objects_) {
    //  destroy object
    //scene_node_->detachObject(obj.object);
    scene_manager_->destroyManualObject(obj.object);
    //  destroy texture
    if (!obj.texture.isNull()) {
      Ogre::TextureManager::getSingleton().remove(obj.texture->getName());
    }
    //  destroy material
    if (!obj.material.isNull()) {
      Ogre::MaterialManager::getSingleton().remove(obj.material->getName());
    }
  }
  objects_.clear();
}

void LoadImages::update(float, float) {
  //  creates all geometry, if necessary
  assembleScene();
  //  draw
  context_->queueRender();  // klappt auch ohne!
}

void LoadImages::loadImagery() {
  //  cancel current imagery, if any
  loader_.reset();

  try {
        loader_.reset(new TileLoader(this)); //hiermit hängt sich das program auf!!
  } catch (std::exception &e) {
    setStatus(StatusProperty::Error, "Message", QString(e.what()));
    return;
  }

  QObject::connect(loader_.get(), SIGNAL(errorOcurred(QString)), this,
                   SLOT(errorOcurred(QString)));
  QObject::connect(loader_.get(), SIGNAL(finishedLoading()), this,
                   SLOT(finishedLoading()));
  // QObject::connect(loader_.get(), SIGNAL(initiatedRequest(QNetworkRequest)), this,
  //                  SLOT(initiatedRequest(QNetworkRequest)));
  QObject::connect(loader_.get(), SIGNAL(receivedImage(QNetworkRequest)), this,
                   SLOT(receivedImage(QNetworkRequest)));
  //  start loading images
  loader_->start();
}

void LoadImages::assembleScene() {
  if (!dirty_) {
    return; //  nothing to update
  }
  dirty_ = false;
  
  if (!loader_) {
    return; //  no tiles loaded, don't do anything
  }
  
  //  get rid of old geometry, we will re-build this
  clearGeometry();
  
 for (const TileLoader::MapTile &tile : loader_->Mytiles()) {
  if (tile.hasImage()) {

    const int w = tile.image().width();  // /2 dass in mitte sitzt!
    const int h = tile.image().height(); // /2 dass in mitte sitzt
    const double tile_w = w * resolution_;
    const double tile_h = h * resolution_;

    double x = tile.posX();
    double y = tile.posY();
    double z = tile.posZ();

    const std::string name_suffix =
        std::to_string(tile.posX()) + "_" + std::to_string(tile.posY()) + "_" +  std::to_string(tile.posZ()) 
         + "_" +  std::to_string(tile.qx()) + "_" +  std::to_string(tile.qy()) + "_" +  std::to_string(tile.qz())
          + "_" +  std::to_string(tile.qw())
        +std::to_string(map_id_) + "_" + std::to_string(scene_id_);

      //  one material per texture
      Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
          "material_" + name_suffix,
          Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
      material->setReceiveShadows(false);
      material->getTechnique(0)->setLightingEnabled(false);
      material->setDepthBias(-16.0f, 0.0f);
      material->setCullingMode(Ogre::CULL_NONE);
      material->setDepthWriteEnabled(false);

      //  create textureing unit
      Ogre::Pass *pass = material->getTechnique(0)->getPass(0);
      Ogre::TextureUnitState *tex_unit = nullptr;
      if (pass->getNumTextureUnitStates() > 0) {
        tex_unit = pass->getTextureUnitState(0);
      } else {
        tex_unit = pass->createTextureUnitState();
      }

      //  only add if we have a texture for it
      Ogre::TexturePtr texture =
          textureFromImage(tile.image(), "texture_" + name_suffix);

      tex_unit->setTextureName(texture->getName());
      tex_unit->setTextureFiltering(Ogre::TFO_BILINEAR);

      //  create an object
      const std::string obj_name = "object_" + name_suffix;
      Ogre::ManualObject *obj = scene_manager_->createManualObject(obj_name);

      Ogre::SceneNode* my_scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode((Ogre::Vector3(0,0,0)));
      my_scene_node_->attachObject(obj);

      //  configure depth & alpha properties
      if (alpha_ >= 0.9998) {
        material->setDepthWriteEnabled(!draw_under_);
        material->setSceneBlending(Ogre::SBT_REPLACE);
      } else {
        material->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        material->setDepthWriteEnabled(false);
      }

      if (draw_under_) {
        obj->setRenderQueueGroup(Ogre::RENDER_QUEUE_3);
      } else {
        obj->setRenderQueueGroup(Ogre::RENDER_QUEUE_MAIN);
      }

      tex_unit->setAlphaOperation(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL,
                                  Ogre::LBS_CURRENT, alpha_);


// see: http://wiki.ogre3d.org/Intermediate+Tutorial+5
// http://wiki.ogre3d.org/Quaternion+and+Rotation+Primer (table of quaternions)
// only works for a quad!!! not for a rectangle :) 

      //  create a quad for this tile
      obj->begin(material->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);

      //  bottom left
      obj->position(0, 0, 0);
      obj->textureCoord(0.0f, 0.0f);
      obj->normal(0.0f, 0.0f, 1.0f);

      // top right
      obj->position(0 + tile_w, 0 + tile_h, 0);
      obj->textureCoord(1.0f, 1.0f);
      obj->normal(0.0f, 0.0f, 1.0f);

      // top left
      obj->position(0, 0 + tile_h, -0);
      obj->textureCoord(0.0f, 1.0f);
      obj->normal(0.0f, 0.0f, 1.0f);

      //  bottom left
      obj->position(0, 0, 0);
      obj->textureCoord(0.0f, 0.0f);
      obj->normal(0.0f, 0.0f, 1.0f);

      // bottom right
      obj->position(0+ tile_w, 0, 0);
      obj->textureCoord(1.0f, 0.0f);
      obj->normal(0.0f, 0.0f, 1.0f);

      // top right
      obj->position(0 + tile_w, 0 + tile_h, 0);
      obj->textureCoord(1.0f, 1.0f);
      obj->normal(0.0f, 0.0f, 1.0f);

   obj->end();

      std::cout<<x<<" "<<y<<" "<<z<<std::endl;
      // not exactly shure how to remove the offset! TODO TODO
      // what works for some rotations:       Ogre::Vector3 position_s(x+tile_w/2, y, z-tile_w/2);  
      Ogre::Vector3 position_s(x, y, z);  
      my_scene_node_->setPosition(position_s);

      Ogre::Quaternion quat;
      quat.x = tile.qx();
      quat.y = tile.qy();
      quat.z = tile.qz();
      quat.w = tile.qw();
      std::cout<<"set quat to: "<<quat<<std::endl;
      my_scene_node_->setOrientation(quat);


      if (draw_under_property_->getValue().toBool()) {
        //  render under everything else
        obj->setRenderQueueGroup(Ogre::RENDER_QUEUE_3);
      }

      // store values in a struct MapObject
      MapObject object;
      object.object = obj;
      object.texture = texture;
      object.material = material;
      objects_.push_back(object); // important for deleting when updating!
    }  // end if tile has img
 }
 scene_id_++;
}

void LoadImages::receivedImage(QNetworkRequest request) {
  ROS_DEBUG("Loaded tile %s", qPrintable(request.url().toString()));
}

void LoadImages::finishedLoading() {
  ROS_INFO("Finished loading all tiles.");
  dirty_ = true;
  setStatus(StatusProperty::Ok, "Message", "Loaded all tiles.");
}

void LoadImages::errorOcurred(QString description) {
  ROS_ERROR("Error: %s", qPrintable(description));
  setStatus(StatusProperty::Error, "Message", description);
}

void LoadImages::reset() {
  Display::reset();
}

} // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::LoadImages, rviz::Display)
