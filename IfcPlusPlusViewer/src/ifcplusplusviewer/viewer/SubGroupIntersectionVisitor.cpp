
#include "SubGroupIntersectionVisitor.h"

SubGroupIntersectionVisitor::SubGroupIntersectionVisitor(osgUtil::Intersector* intersector, ReadCallback* readCallback)
	: osgUtil::IntersectionVisitor( intersector, readCallback )
{
}

void SubGroupIntersectionVisitor::apply(osg::Camera& camera, osg::Group* grp)
{
	// OSG_NOTICE<<"apply(Camera&)"<<std::endl;

	// note, commenting out right now because default Camera setup is with the culling active.  Should this be changed?
	// if (!enter(camera)) return;

	// OSG_NOTICE<<"inside apply(Camera&)"<<std::endl;

	osg::RefMatrix* projection = NULL;
	osg::RefMatrix* view = NULL;
	osg::RefMatrix* model = NULL;

	if (camera.getReferenceFrame()==osg::Transform::RELATIVE_RF && getProjectionMatrix() && getViewMatrix())
	{
		if (camera.getTransformOrder()==osg::Camera::POST_MULTIPLY)
		{
			projection = new osg::RefMatrix(*getProjectionMatrix()*camera.getProjectionMatrix());
			view = new osg::RefMatrix(*getViewMatrix()*camera.getViewMatrix());
			model = new osg::RefMatrix(*getModelMatrix());
		}
		else // pre multiply
		{
			projection = new osg::RefMatrix(camera.getProjectionMatrix()*(*getProjectionMatrix()));
			view = new osg::RefMatrix(*getViewMatrix());
			model = new osg::RefMatrix(camera.getViewMatrix()*(*getModelMatrix()));
		}
	}
	else
	{
		// an absolute reference frame
		projection = new osg::RefMatrix(camera.getProjectionMatrix());
		view = new osg::RefMatrix(camera.getViewMatrix());
		model =  new osg::RefMatrix();
	}

	if (camera.getViewport()) pushWindowMatrix( camera.getViewport() );
	pushProjectionMatrix(projection);
	pushViewMatrix(view);
	pushModelMatrix(model);

	// now push an new intersector clone transform to the new local coordinates
	push_clone();

	grp->traverse( *this );
	//traverse(camera);
	//traverse(grp);

	// pop the clone.
	pop_clone();

	popModelMatrix();
	popViewMatrix();
	popProjectionMatrix();
	if (camera.getViewport()) popWindowMatrix();

	// leave();
}
