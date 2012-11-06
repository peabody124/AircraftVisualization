#include <pthread.h>
#include <osg/PointSprite>
#include <osg/BlendFunc>
#include <osg/StateAttribute>
#include <osg/Point>
#include <osg/Geometry>
#include <osg/Texture2D>
#include <osg/TexEnv>
#include <osg/GLExtensions>
#include <osg/Material>
#include <osg/TexEnv>
#include <osg/PositionAttitudeTransform>
#include <osg/ShapeDrawable>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <osgUtil/Optimizer>

#include <osgViewer/CompositeViewer>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgGA/NodeTrackerManipulator>
#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>

#include <iostream>

#include <osgEarth/MapNode>
#include <osgEarth/XmlUtils>
#include <osgEarth/Viewpoint>

#include <osgEarthSymbology/Color>

#include <osgEarthAnnotation/AnnotationRegistry>
#include <osgEarthAnnotation/AnnotationData>
#include <osgEarthAnnotation/Decluttering>

#include <osgEarthDrivers/kml/KML>
#include <osgEarthDrivers/ocean_surface/OceanSurface>
#include <osgEarthDrivers/cache_filesystem/FileSystemCache>

#include <osgEarthUtil/EarthManipulator>
#include <osgEarthUtil/AutoClipPlaneHandler>
#include <osgEarthUtil/Controls>
#include <osgEarthUtil/SkyNode>
#include <osgEarthUtil/LatLongFormatter>
#include <osgEarthUtil/MouseCoordsTool>
#include <osgEarthUtil/ObjectLocator>

using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;
using namespace osgEarth::Symbology;
using namespace osgEarth::Drivers;
using namespace osgEarth::Annotation;

//! Magic value to check the static data
enum magic_value { MAGIC_VALUE = 0xAF01BC32 };

//! Data preserved between runs
struct global_struct {
    //! The composite viewer for the scene
    osg::ref_ptr<osgViewer::CompositeViewer> viewer;

    //! The position that locates the UAV
    osg::ref_ptr<osgEarth::Util::ObjectLocatorNode> uavPos;
    //! The attitude of the UAV
    osg::ref_ptr<osg::MatrixTransform> uavAttitudeAndScale;
    //! The UAV node
    osg::ref_ptr<osg::Node> uav;

    osg::ref_ptr<osgEarth::MapNode> mapNode;

    //! The tracker which makes the FPV camera track the UAV
    osg::ref_ptr<osgGA::NodeTrackerManipulator> tracker;

    //! The manipulator for the main view
    osg::ref_ptr<EarthManipulator> manip;

    //! Handle to the visualization thread
    pthread_t vis_thread;

    enum magic_value magic;
};

/**
 * Set up the display window so it can be resized.  Default to full scale
 */
void singleWindow(osgViewer::CompositeViewer *viewer, osg::Node *root)
{
    osg::GraphicsContext::WindowingSystemInterface* wsi = osg::GraphicsContext::getWindowingSystemInterface();
    
    unsigned int width, height;
    wsi->getScreenResolution(osg::GraphicsContext::ScreenIdentifier(0), width, height);

    width *= 0.9;
    height *= 0.9;

    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->x = 0;
    traits->y = 0;
    traits->width = width;
    traits->height = height;
    traits->windowDecoration = true;
    traits->doubleBuffer = true;
    traits->sharedContext = 0;

    osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
    if (gc.valid())
    {
        osg::notify(osg::INFO)<<"  GraphicsWindow has been created successfully."<<std::endl;

        // need to ensure that the window is cleared make sure that the complete window is set the correct colour
        // rather than just the parts of the window that are under the camera's viewports
        gc->setClearColor(osg::Vec4f(0.2f,0.2f,0.6f,1.0f));
        gc->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }
    else
    {
        osg::notify(osg::NOTICE)<<"  GraphicsWindow has not been created successfully."<<std::endl;
    }

    /********* First view ***********/
    {
        osgViewer::View *view = new osgViewer::View;
        view->setName("Chase cam");
        osg::ref_ptr<osg::Camera> cam = view->getCamera();
        cam->setGraphicsContext(gc.get());
        cam->setViewport(0, 0, width, height);
        viewer->addView(view);

        view->setSceneData(root);
        view->setCameraManipulator(new osgGA::TrackballManipulator);
        // add the state manipulator
        osg::ref_ptr<osgGA::StateSetManipulator> statesetManipulator = new osgGA::StateSetManipulator;
        statesetManipulator->setStateSet(view->getCamera()->getOrCreateStateSet());

        view->addEventHandler( statesetManipulator.get() );
    }
    /********* Second view ***********/
    {
        osgViewer::View* view = new osgViewer::View;
        view->setName("Camera view");
        osg::ref_ptr<osg::Camera> cam = view->getCamera();
        cam->setGraphicsContext(gc.get());
        cam->setViewport(0, 0, width/3, height/3);

        viewer->addView(view);

        view->setSceneData(root);

        view->addEventHandler( new osgViewer::StatsHandler );
    }

    return;
}


osg::Node* createAirplane(struct global_struct *g)
{
    osg::ref_ptr<osg::Group> model = new osg::Group;

    g->uav = osgDB::readNodeFile("/Users/Cotton/Programming/OpenPilot/artwork/3D Model/multi/joes_cnc/J14-QT_+.3DS");

    if(g->uav) {
        g->uavAttitudeAndScale = new osg::MatrixTransform();
        g->uavAttitudeAndScale->setMatrix(osg::Matrixd::scale(0.2e0,0.2e0,0.2e0));

        // Apply a rotation so model is NED before any other rotations
        osg::MatrixTransform *rotateModelNED = new osg::MatrixTransform();
        rotateModelNED->setMatrix(osg::Matrixd::scale(0.05e0,0.05e0,0.05e0) * osg::Matrixd::rotate(M_PI, osg::Vec3d(0,0,1)));
        rotateModelNED->addChild( g->uav );

        g->uavAttitudeAndScale->addChild( rotateModelNED );

        model->addChild(g->uavAttitudeAndScale);
    } 

    return model.release();
}

/**
 * Update the position of the UAV
 */
void updatePosition(struct global_struct *g, double *LLA, double *quat)
{
    static float angle = 0;
    g->uavPos->getLocator()->setPosition( osg::Vec3d(LLA[1], LLA[0], LLA[2]) );  // Note this takes longtitude first

    // Set the attitude (reverse the attitude)
    // Have to rotate the axes from OP NED frame to OSG frame (X east, Y north, Z down)
    osg::Quat q(quat[1],quat[2],quat[3],quat[0]);
    osg::Vec3d axis;
    //q.getRotate(angle,axis);
    q.makeRotate(angle, osg::Vec3d(axis[1],axis[0],-axis[2]));

    angle+=0.01;
    if (angle > 360) angle = 0;
    q.makeRotate(angle, osg::Vec3d(0,0,1));
    osg::Matrixd rot = osg::Matrixd::rotate(q);

    g->uavAttitudeAndScale->setMatrix(rot);
}

/**
 * Update the camera angle relative to the body
 */
void updateCamera(struct global_struct *g, float pitch, float roll)
{
    osg::Matrixd yawMat, pitchMat, rollMat;
    
    rollMat.makeRotate(roll / 180.0 * M_PI, osg::Vec3d(0,1,0));    
    pitchMat.makeRotate(-M_PI/2.0 + pitch / 180.0 * M_PI,osg::Vec3d(1,0,0));
    yawMat.makeRotate(M_PI, osg::Vec3d(0,0,1));

    // We need a 180 degree yaw rotation to view "forward" with the quad
    g->tracker->setByMatrix(yawMat * rollMat * pitchMat);
}

/**
 * Initialize the scene and populate a structure that contains
 * all the required handle for the rest of the functionality
 */
struct global_struct * initialize()
{
 // Initialize the global data if it doesn't exist
    struct global_struct *g = (struct global_struct *) malloc(sizeof(struct global_struct));
    g->viewer = NULL;
    g->magic = MAGIC_VALUE;

    // Create a root node
    osg::ref_ptr<osg::Group> root = new osg::Group;

    osg::Node* earth = osgDB::readNodeFile("/Users/Cotton/Programming/osg/osgearth/tests/boston.earth");
    g->mapNode = osgEarth::MapNode::findMapNode( earth );

    osg::Node* airplane = createAirplane(g);
    g->uavPos = new osgEarth::Util::ObjectLocatorNode(g->mapNode->getMap());
    g->uavPos->getLocator()->setPosition( osg::Vec3d(-71.100549, 42.349273, 200) );
    g->uavPos->addChild(airplane);

    root->addChild(earth);
    root->addChild(g->uavPos);

    osgUtil::Optimizer optimizer;
    optimizer.optimize(root);

    if (g->viewer == NULL)
        g->viewer = new osgViewer::CompositeViewer();
    singleWindow(g->viewer, root);

    g->viewer->getView(0)->addEventHandler(new osgViewer::StatsHandler);
    g->viewer->getView(0)->addEventHandler(new osgViewer::ThreadingHandler);
    
    g->viewer->realize();

    osg::Matrix projectionOffset;
    osg::Matrix viewOffset;

    g->manip = new EarthManipulator();
    g->viewer->getView(0)->setCameraManipulator(g->manip);
    g->manip->setViewpoint( Viewpoint(-71.100549, 42.349273, 200, 180, -25, 350.0), 10.0 );
    g->manip->setTetherNode(g->uavPos);

    g->tracker = new osgGA::NodeTrackerManipulator();
    g->tracker->setTrackerMode( osgGA::NodeTrackerManipulator::NODE_CENTER_AND_ROTATION );
    g->tracker->setNode(g->uav);
    g->tracker->setTrackNode(g->uav);
    g->tracker->setMinimumDistance(100); 
    g->tracker->setDistance(500); 
    g->tracker->setElevation(300); 
    g->tracker->setHomePosition( osg::Vec3f(0.f,40.f,40.f), osg::Vec3f(0.f,0.f,0.f), osg::Vec3f(0,0,1) ); 
    g->viewer->getView(1)->setCameraManipulator(g->tracker);

    return g;
}

void *run_thread(void * arg)
{
    struct global_struct *g = (struct global_struct *) arg;
    
    double LLA[] = {42.349273, -71.100549, 500};
    double quat[] = {1,0,0,0};
    double pitch = -45;

    while(!g->viewer->done())
    {
        pitch+=0.1;
        if (pitch > 45)
            pitch = -45;

        updatePosition(g, LLA, quat);
        updateCamera(g, pitch, 20);
        LLA[1] += 0.00001;

        g->viewer->frame();
    }
    return NULL;
}

/**
 * Create a thread to run the visualization
 */
void startRunThread(global_struct *g)
{
     // Set up the thread parameters
    pthread_attr_t xThreadAttributes;
    pthread_attr_init( &xThreadAttributes );
    pthread_attr_setdetachstate( &xThreadAttributes, PTHREAD_CREATE_DETACHED );

    pthread_create(&g->vis_thread, &xThreadAttributes, run_thread, g);   
}

void shutdown(struct global_struct *g)
{
  // TODO: Clean up weak pointers properly
    /*
  //! The composite viewer for the scene
    //g->viewer->release();

    free(g->uavPos);
    free(g->uavAttitudeAndScale);
    free(g->uav);
    free(g->mapNode);
    free(g->tracker);
    free(g->manip);

    // Release the viewer
    delete(g->viewer);
    g->viewer = NULL;
    */
    free(g);
}

/**
 * The matlab entry function
 */
int main()
{

    struct global_struct *g = initialize();

    startRunThread(g);

    while(1);
    pthread_join(g->vis_thread, NULL);

    shutdown(g);

    return 0;
}

