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
  File: map_annotation_tool.h
  Purpose: RViz plugin for annotating maps
  @author Ahmed Faisal Abdelrahman
  @author Sushant Vijay Chavan
  @version 1.0 28/12/18
*/

#ifndef MAP_ANNOTATION_TOOL_H
#define MAP_ANNOTATION_TOOL_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <QObject>

# include <ros/ros.h>

# include "rviz/default_plugin/tools/pose_tool.h"

#include <fstream>
#include <string>
#include <vector>

#endif

namespace rviz
{
class Arrow;
class DisplayContext;
class StringProperty;
class VectorProperty;
class FloatProperty;
class Property;
}

namespace Map_Annotation_Tool
{

class MapAnnotationTool: public rviz::PoseTool
{
  Q_OBJECT
public:
  MapAnnotationTool();
  virtual ~MapAnnotationTool();
  virtual void onInitialize();

protected:
  virtual void onPoseSet(double x, double y, double theta);
  virtual void addMarker(const std::string& name, double x, double y,
      double theta);
  virtual void addMarkerProperty(const std::string& name, double x, double y,
      double theta);
  virtual Ogre::Quaternion computeQuaternion(double theta);

  virtual void generateMarkersFromPoses(const std::vector<std::string>& names,
      const std::vector<std::vector<float> >& poses, bool addProperties = true);

  virtual void loadPosesFromFile();
  virtual void savePosesToFile();

  virtual std::string getPathToResourceFile();

  virtual void setupRootProperties();

private Q_SLOTS:
  void updateMarker();

private:
  std::vector<rviz::Arrow*> markerList_;
  rviz::Property* markerContainerProperty_;
  float markerScale_;
  std::string path_;
};

}

#endif

