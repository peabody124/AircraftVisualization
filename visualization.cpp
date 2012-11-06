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
    int runs;
    osgViewer::CompositeViewer *viewer;
    enum magic_value magic;
};

//! Pointer to the global data
static struct global_struct  *global = NULL;

//! Class that handles clicking objects in geometry
class KeyEventHandler : public osgGA::GUIEventHandler
{
protected:
   osgViewer::Viewer *m_viewer;
 
public:
   KeyEventHandler(osgViewer::Viewer *node) : osgGA::GUIEventHandler()
   {
       m_viewer = node;
   }

   virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
   {
       switch (ea.getEventType())
       {
           case(osgGA::GUIEventAdapter::PUSH):
           break;
           default:
               return false;
       }
       return true;
   }
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
        cam->setViewport(0, 0, width/2, height/2);
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
        cam->setViewport(width/2, 0, width/2, height/2);

        viewer->addView(view);

        view->setSceneData(root);
        view->setCameraManipulator(new osgGA::TrackballManipulator);

        view->addEventHandler( new osgViewer::StatsHandler );
    }


/*
    unsigned int numCameras = 2;
    double aspectRatioScale = 1.0;///(double)numCameras;
    osg::ref_ptr<osg::Camera> camera;
    for(unsigned int i=0; i<numCameras;++i)
    {
        camera = new osg::Camera;
        camera->setGraphicsContext(gc.get());
        camera->setViewport(new osg::Viewport((i*width)/numCameras,(i*height)/numCameras, width/numCameras, height/numCameras));
        GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
        camera->setDrawBuffer(buffer);
        camera->setReadBuffer(buffer);

        if (i == 0) 
            viewer->addSlave(camera.get(), osg::Matrixd(), osg::Matrixd::scale(aspectRatioScale,1.0,1.0));
    } */

    return;
}

osg::MatrixTransform* uavAttitudeAndScale;
osgEarth::MapNode* mapNode;
osgEarth::Util::ObjectLocatorNode* uavPos;

osg::Node* createAirplane()
{
    osg::Group* model = new osg::Group;
    osg::Node *uav;

    uav = osgDB::readNodeFile("/Users/Cotton/Programming/OpenPilot/artwork/3D Model/multi/joes_cnc/J14-QT_+.3DS");

    if(uav) {
        uavAttitudeAndScale = new osg::MatrixTransform();
        uavAttitudeAndScale->setMatrix(osg::Matrixd::scale(0.2e0,0.2e0,0.2e0));

        // Apply a rotation so model is NED before any other rotations
        osg::MatrixTransform *rotateModelNED = new osg::MatrixTransform();
        rotateModelNED->setMatrix(osg::Matrixd::scale(0.05e0,0.05e0,0.05e0) * osg::Matrixd::rotate(M_PI, osg::Vec3d(0,0,1)));
        rotateModelNED->addChild( uav );

        uavAttitudeAndScale->addChild( rotateModelNED );

        model->addChild(uavAttitudeAndScale);
    } 

    return model;
}

/*
osg::Camera* createCamera( int x, int y, int w, int h, const std::string& name="", bool windowDecoration=false )
{
    osg::DisplaySettings* ds = osg::DisplaySettings::instance().get();
    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->windowName = name;
    traits->windowDecoration = windowDecoration;
    traits->x = x;
    traits->y = y;
    traits->width = w;
    traits->height = h;
    traits->doubleBuffer = true;
    traits->alpha = ds->getMinimumNumAlphaBits();
    traits->stencil = ds->getMinimumNumStencilBits();
    traits->sampleBuffers = ds->getMultiSamples();
    traits->samples = ds->getNumMultiSamples();

    osg::ref_ptr<osg::Camera> camera = new osg::Camera;
    camera->setGraphicsContext( new osgQt::GraphicsWindowQt(traits.get()) );

    camera->setClearColor( osg::Vec4(0.2, 0.2, 0.6, 1.0) );
    camera->setViewport( new osg::Viewport(0, 0, traits->width, traits->height) );
    camera->setProjectionMatrixAsPerspective(
                30.0f, static_cast<double>(traits->width)/static_cast<double>(traits->height), 1.0f, 10000.0f );
    return camera.release();
}
*/

/**
 * Update the position of the UAV
 */
void updatePosition(double *LLA, double *quat)
{
    uavPos->getLocator()->setPosition( osg::Vec3d(LLA[1], LLA[0], LLA[2]) );  // Note this takes longtitude first

    // Set the attitude (reverse the attitude)
    // Have to rotate the axes from OP NED frame to OSG frame (X east, Y north, Z down)
    osg::Quat q(quat[1],quat[2],quat[3],quat[0]);
    double angle;
    osg::Vec3d axis;
    q.getRotate(angle,axis);
    q.makeRotate(angle, osg::Vec3d(axis[1],axis[0],-axis[2]));
    osg::Matrixd rot = osg::Matrixd::rotate(q);

    uavAttitudeAndScale->setMatrix(rot);
}

/**
 * The matlab entry function
 */
int main()
{
    // Initialize the global data if it doesn't exist
    global = (struct global_struct *) malloc(sizeof(*global));
    global->runs = 0;
    global->viewer = NULL;
    global->magic = MAGIC_VALUE;

    // Create a root node
    osg::ref_ptr<osg::Group> root = new osg::Group;

    osg::Node* earth = osgDB::readNodeFile("/Users/Cotton/Programming/osg/osgearth/tests/boston.earth");
    mapNode = osgEarth::MapNode::findMapNode( earth );

    osg::Node* airplane = createAirplane();
    uavPos = new osgEarth::Util::ObjectLocatorNode(mapNode->getMap());
    uavPos->getLocator()->setPosition( osg::Vec3d(-71.100549, 42.349273, 200) );
    uavPos->addChild(airplane);

    root->addChild(earth);
    root->addChild(uavPos);

    if (global->viewer == NULL)
        global->viewer = new osgViewer::CompositeViewer();
    singleWindow(global->viewer, root);

    osg::ref_ptr<EarthManipulator> manip = new EarthManipulator();

/*    global->viewer->addEventHandler(new osgViewer::StatsHandler);
    global->viewer->addEventHandler(new osgViewer::ThreadingHandler);
    global->viewer->addEventHandler(new KeyEventHandler(global->viewer)); 
    global->viewer->setSceneData(root); */
    
    global->viewer->realize();

    osg::Matrix projectionOffset;
    osg::Matrix viewOffset;

    global->viewer->getView(0)->setCameraManipulator(manip);
    manip->setViewpoint( Viewpoint(-71.100549, 42.349273, 200, 180, -25, 350.0), 10.0 );
    manip->setTetherNode(uavPos);

    //uavCamera->setView(global->viewer);
    //global->viewer->addSlave(uavCamera);

    double LLA[] = {42.349273, -71.100549, 100};
    double quat[] = {1,0,0,0};

    double pitch = -45;
    while(!global->viewer->done())
    {

        updatePosition(LLA, quat);
        LLA[2] += 0.01;

        osg::Matrix pos;
        uavPos->getLocator()->getPositionMatrix(pos);
        global->viewer->frame();
    }

    // Release the viewer
    delete(global->viewer);
    global->viewer = NULL;
    
    return 0;
}

