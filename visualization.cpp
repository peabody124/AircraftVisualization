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
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/NodeTrackerManipulator>
#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
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
    osgViewer::Viewer *viewer;
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
void singleWindow(osgViewer::Viewer *viewer)
{
    osg::GraphicsContext::WindowingSystemInterface* wsi = osg::GraphicsContext::getWindowingSystemInterface();
    if (!wsi) 
    {
        osg::notify(osg::NOTICE)<<"Error, no WindowSystemInterface available, cannot create windows."<<std::endl;
        return;
    }
    
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

    unsigned int numCameras = 1;
    double aspectRatioScale = 1.0;///(double)numCameras;
    for(unsigned int i=0; i<numCameras;++i)
    {
        osg::ref_ptr<osg::Camera> camera = new osg::Camera;
        camera->setGraphicsContext(gc.get());
        camera->setViewport(new osg::Viewport((i*width)/numCameras,(i*height)/numCameras, width/numCameras, height/numCameras));
        GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
        camera->setDrawBuffer(buffer);
        camera->setReadBuffer(buffer);

        viewer->addSlave(camera.get(), osg::Matrixd(), osg::Matrixd::scale(aspectRatioScale,1.0,1.0));
    }
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
    osg::Group *root = new osg::Group;

    // Plot the cells
    osg::ref_ptr<osg::Group> group = new osg::Group;

    osg::Node* earth = osgDB::readNodeFile("/Users/Cotton/Programming/osg/osgearth/tests/boston.earth");
    //mapNode = osgEarth::MapNode::findMapNode( earth );

    root->addChild(earth);

    osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform();
    pat->addChild(group);
    pat->setScale(osg::Vec3(1,1,2));

    if (global->viewer == NULL)
        global->viewer = new osgViewer::Viewer();
    singleWindow(global->viewer);

    osg::ref_ptr<EarthManipulator> manip = new EarthManipulator();

    global->viewer->setCameraManipulator( manip );   
    
    global->viewer->addEventHandler(new osgViewer::StatsHandler);
    global->viewer->addEventHandler(new osgViewer::ThreadingHandler);
    global->viewer->addEventHandler(new KeyEventHandler(global->viewer));
    global->viewer->setSceneData(root);
    
    global->viewer->realize();
    while(!global->viewer->done())
    {
        global->viewer->frame();
    }

    // Release the viewer
    delete(global->viewer);
    global->viewer = NULL;
    
    return 0;
}

