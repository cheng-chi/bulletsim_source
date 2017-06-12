#pragma once
#include <osg/Transform>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <iostream>

class SetColorsVisitor : public osg::NodeVisitor
{

public:
  osg::ref_ptr<osg::Vec4Array> colors;

  SetColorsVisitor(float r, float g, float b, float a ) {
    osg::Vec4 color(r,g,b,a);
    colors = new osg::Vec4Array;
    colors->push_back(color);
    setTraversalMode(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN );
  }
  SetColorsVisitor(const osg::Vec4f& color) {
    colors = new osg::Vec4Array;
    colors->push_back(color);
    setTraversalMode(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN );
  }

  void apply( osg::Geode& geode ) {
    for (int i=0; i < geode.getNumDrawables(); i++) {
      applyDrawable(geode.getDrawable(i));
    }
  }

protected:
  void applyDrawable( osg::Drawable* drawable ) {
    osg::Geometry* geom = drawable->asGeometry();
    if (geom) {
      geom->setColorArray(colors.get());
      //geom->setColorBinding(osg::Geometry::BIND_OVERALL);
    }
    osg::ShapeDrawable* sd = dynamic_cast<osg::ShapeDrawable*>(drawable);
    if (sd != 0) {
      sd->setColor(colors->at(0));
    }

  }
};
