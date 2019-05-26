/*
 * Copyright © 2018 Ahmed Faisal Abdelrahman, Sushant Vijay Chavan All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright notice, this
 *       list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright notice, this
 *       list of conditions and the following disclaimer in the documentation and/or
 *       other materials provided with the distribution.
 *     * Neither the name of “Hochschule Bonn-Rhein-Sieg” nor the names of its contributors
 *       may be used to endorse or promote products derived from this software without specific
 *       prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
  File: map_annotation_tool.cpp
  Purpose: RViz plugin for annotating maps
  @author Ahmed Faisal Abdelrahman
  @author Sushant Vijay Chavan
  @version 1.0 28/12/18
*/

#include <OgreSceneNode.h>
#include <QString>
#include <sstream>

#include "rviz/ogre_helpers/arrow.h"
#include "rviz/ogre_helpers/movable_text.h"
#include "rviz/properties/string_property.h"
#include <rviz/properties/vector_property.h>
#include <rviz/properties/float_property.h>

#include "ros/package.h"
#include "yaml-cpp/yaml.h"

#include "map_annotation_tool.h"

using namespace rviz;
using namespace std;
using namespace Map_Annotation_Tool;

MapAnnotationTool::MapAnnotationTool()
: markerScale_(0.25)
{
  shortcut_key_ = 'l';
}

MapAnnotationTool::~MapAnnotationTool()
{
}

void MapAnnotationTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("Map Annotation Tool");
  setupRootProperties();
  loadPosesFromFile();
}

void MapAnnotationTool::setupRootProperties()
{
  markerContainerProperty_ = new rviz::Property(
    QString::fromStdString("MarkerContainer"));
  getPropertyContainer()->addChild(markerContainerProperty_);

  rviz::FloatProperty* markerSizeProperty = new rviz::FloatProperty(
    QString::fromStdString(std::string("Marker Size")), static_cast<float>(markerScale_),
    "Set the scaling factor to change marker size", getPropertyContainer(), SLOT(updateMarker()), this);
  markerSizeProperty->setReadOnly(false);
}

std::string MapAnnotationTool::getPathToResourceFile()
{
  std::stringstream ss;
  ss << ros::package::getPath("mcr_default_env_config") << "/";
  ss << std::getenv("ROBOT_ENV") << "/";
  ss << "navigation_goals.yaml";

  return ss.str();
}

void MapAnnotationTool::loadPosesFromFile()
{
  YAML::Node goals = YAML::LoadFile(getPathToResourceFile());
  std::vector < std::string > names;
  std::vector < std::vector<float> > poses;

  for (const auto& kv : goals)
  {
    std::string currentName = kv.first.as<std::string>();

    names.push_back(currentName);
    poses.push_back(goals[currentName.c_str()].as<std::vector<float> >());
  }

  generateMarkersFromPoses(names, poses);
}

void MapAnnotationTool::savePosesToFile()
{
  YAML::Node mainNode;
  rviz::Property* markerContainer = getPropertyContainer()->childAt(0);

  for (int i = 0; i < markerContainer->numChildren(); i++)
  {
    std::string name =
        dynamic_cast<rviz::StringProperty*>(markerContainer->childAt(i)->childAt(
            0))->getString().toUtf8().constData();
    Ogre::Vector3 vec =
        dynamic_cast<rviz::VectorProperty*>(markerContainer->childAt(i)->childAt(
            1))->getVector();
    float orientation =
        dynamic_cast<rviz::FloatProperty*>(markerContainer->childAt(i)->childAt(2))->getFloat();

    std::vector<float> pose;
    pose.push_back(vec[0]);
    pose.push_back(vec[1]);
    pose.push_back(orientation);
    mainNode[name.c_str()] = pose;
  }

  std::ofstream fout(getPathToResourceFile());
  fout << mainNode;
}

void MapAnnotationTool::generateMarkersFromPoses(
    const std::vector<std::string>& names,
    const std::vector<std::vector<float> >& poses, bool addProperties)
{
  for (int i = 0; i < poses.size(); i++)
  {
    double x = poses[i][0];
    double y = poses[i][1];
    double theta = poses[i][poses[i].size() - 1];
    addMarker(names[i], x, y, theta);

    if (addProperties)
    {
      addMarkerProperty(names[i], x, y, theta);
    }
  }
}

void MapAnnotationTool::updateMarker()
{
  std::vector < std::vector<float> > updatedPoses;
  std::vector < std::string > updatedNames;
  rviz::Property* markerContainer = getPropertyContainer()->childAt(0);
  for (int i = 0; i < markerContainer->numChildren(); i++)
  {
    updatedNames.push_back(
        dynamic_cast<rviz::StringProperty*>(markerContainer->childAt(i)->childAt(
            0))->getString().toUtf8().constData());

    std::vector<float> newPose;
    Ogre::Vector3 vec =
        dynamic_cast<rviz::VectorProperty*>(markerContainer->childAt(i)->childAt(
            1))->getVector();
    for (int j = 0; j < 2; j++)
    {
      newPose.push_back(vec[j]);
    }

    float orientation =
        dynamic_cast<rviz::FloatProperty*>(markerContainer->childAt(i)->childAt(2))->getFloat();
    newPose.push_back(orientation);

    updatedPoses.push_back(newPose);
  }

  for (int i = 0; i < markerList_.size(); i++)
  {
    delete markerList_[i];
  }
  markerList_.clear();

  markerScale_ = dynamic_cast<rviz::FloatProperty*>(getPropertyContainer()->childAt(1))->getFloat();

  generateMarkersFromPoses(updatedNames, updatedPoses, false);
  savePosesToFile();
}

void MapAnnotationTool::onPoseSet(double x, double y, double theta)
{
  std::stringstream ss;
  ss << "Pose " << getPropertyContainer()->childAt(0)->numChildren() + 1;
  std::string positionName = ss.str();

  addMarker(positionName, x, y, theta);
  addMarkerProperty(positionName, x, y, theta);
  savePosesToFile();
}

Ogre::Quaternion MapAnnotationTool::computeQuaternion(double theta)
{
  // Adaptation of PoseTool::processMouseEvent() taken from
  // https://github.com/ros-visualization/rviz/blob/melodic-devel/src/rviz/default_plugin/tools/pose_tool.cpp

  //we need base_orient, since the arrow goes along the -z axis by default
  Ogre::Quaternion orient_x = Ogre::Quaternion(
      Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y);

  return Ogre::Quaternion(Ogre::Radian(theta), Ogre::Vector3::UNIT_Z) * orient_x;
}

void MapAnnotationTool::addMarker(const std::string& name, double x, double y,
    double theta)
{
  // ROS_INFO("Creating marker: Position(%.3f, %.3f), Angle: %.3f\n", x, y, theta);

  Ogre::Quaternion quat = computeQuaternion(theta);

  Arrow* marker = new Arrow(scene_manager_, NULL, markerScale_ * 2.0, markerScale_ * 0.2, markerScale_ * 0.5f, markerScale_ * 0.35f);
  marker->setColor(1.0f, 0.0f, 0.0f, 1.0f);
  marker->setPosition(Ogre::Vector3(x, y, 0.0));
  marker->setOrientation(quat);
  marker->getSceneNode()->setVisible(true);

  MovableText* caption = new MovableText(name.c_str());
  caption->setTextAlignment(MovableText::H_CENTER, MovableText::V_CENTER);
  caption->setCharacterHeight(0.2);
  caption->setColor(Ogre::ColourValue(0.0, 0.0, 1.0, 1.0));
  caption->setCaption(name.c_str());
  caption->showOnTop(true);
  marker->getSceneNode()->attachObject(caption);

  markerList_.push_back(marker);
}

void MapAnnotationTool::addMarkerProperty(const std::string& name, double x,
    double y, double theta)
{
  rviz::Property* markerProperty = new rviz::Property(
      QString::fromStdString(name));

  rviz::StringProperty* nameProp = new rviz::StringProperty(
      QString::fromStdString(std::string("Name")), QString::fromStdString(name),
      "Marker Name", markerProperty, SLOT(updateMarker()), this);

  rviz::VectorProperty* posProperty = new rviz::VectorProperty(
      QString::fromStdString(std::string("Position")), Ogre::Vector3(x, y, 0.0),
      "Marker Position", markerProperty, SLOT(updateMarker()), this);
  posProperty->setReadOnly(false);

  rviz::FloatProperty* orientationProperty = new rviz::FloatProperty(
      QString::fromStdString(std::string("Orientation (radians)")),
      static_cast<float>(theta),
      "Marker Orientation", markerProperty, SLOT(updateMarker()), this);
  orientationProperty->setReadOnly(false);

  markerContainerProperty_->addChild(markerProperty);
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( Map_Annotation_Tool::MapAnnotationTool, rviz::Tool )
