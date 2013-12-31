
#pragma once
#include <osgUtil/IntersectionVisitor>

class SubGroupIntersectionVisitor : public osgUtil::IntersectionVisitor
{
public:
	SubGroupIntersectionVisitor(osgUtil::Intersector* intersector=0, ReadCallback* readCallback=0);

	void apply(osg::Camera& cam, osg::Group* grp);
};
