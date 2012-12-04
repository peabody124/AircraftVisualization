/**
 ******************************************************************************
 * @author     James Cotton, Copyright (C) 2012.
 *
 * @see        The GNU Public License (GPL) Version 3
 ******************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */


#include <osg/PointSprite>
#include <osg/BlendFunc>
#include <osg/StateAttribute>
#include <osg/Point>
#include <osg/Geometry>
#include <osg/Texture2D>
#include <osg/Depth>
#include <osg/StateSet>
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

#include <iostream>
#include <pthread.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <netinet/in.h>

using namespace osgEarth::Util;
using namespace osgEarth::Util::Controls;
using namespace osgEarth::Symbology;
using namespace osgEarth::Drivers;
using namespace osgEarth::Annotation;
using namespace std;

//! Structure transferred over the network
struct uav_data {
    double q[4]; //[w x y z], which is different from OSG's quat representation, [x y z w]
    double NED[3]; //[m]
    double roll; //[deg]
    double pitch; //[deg]
};

double homeLat=42.349273;
double homeLon=-71.100549;
double homeAlt=0;
double initialAlt=200;
double modelScale=0.002e0;
double initialNED[3]={0,0,0};
string earthFile="osgearth_models/boston.earth";
string worldFile="";
string modelFile="airframe_models/joe_cnc/J14-QT_X.3DS";
bool useOSGEarth = false;

//! Magic value to check the static data
enum magic_value { MAGIC_VALUE = 0xAF01BC32 };

//! Data preserved between runs
struct global_struct {
    //! The composite viewer for the scene
    osg::ref_ptr<osgViewer::CompositeViewer> viewer;

    //! Whether we are rendering in earth or closed scene
    bool earth;

    //! The position that locates the UAV when rendering in earth
    osg::ref_ptr<osgEarth::Util::ObjectLocatorNode> uavPos;

    //! The position that locates the UAV when rendering in simple scenes
    osg::ref_ptr<osg::PositionAttitudeTransform> pat;

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

//! Provide an error message
void diep(char *s)
{
    perror(s);
    exit(1);
}

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

/**
 * Load the UAV model and attach it to a position and rotation node
 */
osg::Node* createAirplane(struct global_struct *g)
{
    osg::ref_ptr<osg::Group> model = new osg::Group;

    cout << "Model: " << modelFile << endl;
    g->uav = osgDB::readNodeFile(modelFile);
    if (g->uav == NULL) {
        diep( (char*) "Dude, update your model file");
    }

    if(g->uav) {
        g->uavAttitudeAndScale = new osg::MatrixTransform();
        g->uavAttitudeAndScale->setMatrix(osg::Matrixd::scale(1,1,1));

        // Apply a rotation so model is NED before any other rotations
        osg::MatrixTransform *rotateModelNED = new osg::MatrixTransform();
        rotateModelNED->setMatrix(osg::Matrixd::scale(modelScale,modelScale,modelScale) * osg::Matrixd::rotate(M_PI, osg::Vec3d(0,0,1)));
        rotateModelNED->addChild( g->uav );

        g->uavAttitudeAndScale->addChild( rotateModelNED );

        model->addChild(g->uavAttitudeAndScale);
    } 

    return model.release();
}

/**
 * Create a simple scene to fly around
 */
osg::Node *makeBase( void )
{
    int i, c;
    float theta;
    float ir = 20.0f;

    osg::Vec3Array *coords = new osg::Vec3Array(19);
    osg::Vec2Array *tcoords = new osg::Vec2Array(19);
    osg::Vec4Array *colors = new osg::Vec4Array(1);

    (*colors)[0].set(1.0f,1.0f,1.0f,1.0f);

    c = 0;
    (*coords)[c].set(0.0f,0.0f,0.0f);
    (*tcoords)[c].set(0.0f,0.0f);
    
    for( i = 0; i <= 18; i++ )
    {
        theta = osg::DegreesToRadians((float)i * 20.0);

        (*coords)[c].set(ir * cosf( theta ), ir * sinf( theta ), 0.0f);
        (*tcoords)[c].set((*coords)[c][0]/36.0f,(*coords)[c][1]/36.0f);

        c++;
    }

    osg::Geometry *geom = new osg::Geometry;

    geom->setVertexArray( coords );

    geom->setTexCoordArray( 0, tcoords );

    geom->setColorArray( colors );
    geom->setColorBinding( osg::Geometry::BIND_OVERALL );

    geom->addPrimitiveSet( new osg::DrawArrays(osg::PrimitiveSet::TRIANGLE_FAN,0,19) );

    osg::Texture2D *tex = new osg::Texture2D;

    tex->setImage(osgDB::readImageFile("Images/water.rgb"));
    tex->setWrap( osg::Texture2D::WRAP_S, osg::Texture2D::REPEAT );
    tex->setWrap( osg::Texture2D::WRAP_T, osg::Texture2D::REPEAT );

    osg::StateSet *dstate = new osg::StateSet;
    dstate->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
    dstate->setTextureAttributeAndModes(0, tex, osg::StateAttribute::ON );

    dstate->setTextureAttribute(0, new osg::TexEnv );

    // clear the depth to the far plane.
    osg::Depth* depth = new osg::Depth;
    depth->setFunction(osg::Depth::ALWAYS);
    depth->setRange(1.0,1.0);   
    dstate->setAttributeAndModes(depth,osg::StateAttribute::ON );

    dstate->setRenderBinDetails(-1,"RenderBin");

    geom->setStateSet( dstate );

    osg::Geode *geode = new osg::Geode;
    geode->addDrawable( geom );

    return geode;
}

/**
 * Update the position of the UAV
 */
void updatePosition(struct global_struct *g, double *NED, double *quat)
{
    double angle = 0;

    if (g->earth) {
        // If flying in virtual earth then convert from NED to LLA
        const double HOME[] = {homeLat, homeLon, homeAlt};
        double T[3];
        double LLA[3];
        double lat, alt;

        lat = HOME[0] * M_PI / 180.0;
        alt = HOME[2];

        T[0] = alt+6.378137E6f;
        T[1] = cos(lat)*(alt+6.378137E6);
        T[2] = -1.0;

        LLA[0] = HOME[0] + NED[0] / T[0] * 180.0 / M_PI;
        LLA[1] = HOME[1] + NED[1] / T[1] * 180.0 / M_PI;
        LLA[2] = HOME[2] + NED[2] / T[2];
        g->uavPos->getLocator()->setPosition( osg::Vec3d(LLA[1], LLA[0], LLA[2]) );  // Note this takes longtitude first
    } else {
        g->pat->setPosition(osg::Vec3d(NED[1], NED[0] , -NED[2])); //Convert to ENU
    }

    // Set the attitude (reverse the attitude)
    // Have to rotate the axes from OP NED frame to OSG frame (X east, Y north, Z up)
    osg::Quat q(quat[1],quat[2],quat[3],quat[0]);
    osg::Vec3d axis;
    q.getRotate(angle,axis);
    q.makeRotate(angle, osg::Vec3d(axis[1],axis[0],-axis[2]));
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

extern osg::Node *makeTerrain( void );
extern osg::Node *makeTrees( void );
extern osg::Node *makeSky( void );

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


    g->earth = useOSGEarth;

    // Create a root node
    osg::ref_ptr<osg::Group> root = new osg::Group;
    if (g->viewer == NULL)
        g->viewer = new osgViewer::CompositeViewer();
    singleWindow(g->viewer, root);

    if (g->earth) {
		 
        cout << "Earth: " << earthFile << endl;

        osg::Node* earth = osgDB::readNodeFile(earthFile);
        if (earth == NULL) {
            diep((char*) "Dude, update your boston.earth path");
        }

        g->mapNode = osgEarth::MapNode::findMapNode( earth );

        osg::Node* airplane = createAirplane(g);
        g->uavPos = new osgEarth::Util::ObjectLocatorNode(g->mapNode->getMap());
        g->uavPos->getLocator()->setPosition( osg::Vec3d(-71.100549, 42.349273, 200) );
        g->uavPos->addChild(airplane);
        root->addChild(earth);
        root->addChild(g->uavPos);

        g->manip = new EarthManipulator();
        g->viewer->getView(0)->setCameraManipulator(g->manip);
        g->manip->setViewpoint( Viewpoint(-71.100549, 42.349273, 200, 180, -25, 350.0), 10.0 );
        g->manip->setTetherNode(g->uavPos);

    } else {
        osg::Node* world = osgDB::readNodeFile(worldFile);

        osg::Node* airplane = createAirplane(g);
        g->pat = new osg::PositionAttitudeTransform();
        g->pat->addChild(airplane);
		 
        osg::ref_ptr<osg::PositionAttitudeTransform> pat = new osg::PositionAttitudeTransform();

        if (world == NULL) { //Default is to use hang glider world taken from OSG examples
            world = makeBase();
			 
            root->addChild(makeSky());
            root->addChild(makeBase());
            root->addChild(makeTerrain());
            root->addChild(makeTrees());
            root->addChild(g->pat);
            g->viewer->getView(0)->setCameraManipulator(new osgGA::TrackballManipulator());			 
        }
        else {
            pat->addChild(world);
            pat->setScale(osg::Vec3d(.0025*200,.0025*200,.0025*200)); //This sets the world scale. It should be adjusted to be as close as possible to 1 meter/unit in OSG
            pat->setPosition(osg::Vec3f(0,0,16*200));
            root->addChild(pat);
        }
        root->addChild(g->pat);
        g->viewer->getView(0)->setCameraManipulator(new osgGA::TrackballManipulator());

        //Convert NED into ENU
        g->pat->setPosition(osg::Vec3d(initialNED[1], initialNED[0], -initialNED[2]));
    }


    osgUtil::Optimizer optimizer;
    optimizer.optimize(root);

    g->viewer->getView(0)->addEventHandler(new osgViewer::StatsHandler);
    g->viewer->getView(0)->addEventHandler(new osgViewer::ThreadingHandler);
    
    g->tracker = new osgGA::NodeTrackerManipulator();
    g->tracker->setTrackerMode( osgGA::NodeTrackerManipulator::NODE_CENTER_AND_ROTATION );
    g->tracker->setNode(g->uav);
    g->tracker->setTrackNode(g->uav);
    g->tracker->setMinimumDistance(100); 
    g->tracker->setDistance(500); 
    g->tracker->setElevation(initialAlt+100); 
    g->tracker->setHomePosition( osg::Vec3f(0.f,40.f,40.f), osg::Vec3f(0.f,0.f,0.f), osg::Vec3f(0,0,1) ); 
    g->viewer->getView(1)->setCameraManipulator(g->tracker);

    return g;
}

/**
 * A thread which gets the data from the simulation over UDP
 */
void *run_thread(void * arg)
{
    struct global_struct *g = (struct global_struct *) arg;

    int s;
    struct sockaddr_in server;
    struct sockaddr client;
    socklen_t client_len = sizeof(client);

    s = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);

    memset(&client,0,sizeof(client));
    memset(&server,0,sizeof(server));
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = inet_addr("127.0.0.1");
    server.sin_port = htons(3000);
    bind(s, (struct sockaddr *)&server,sizeof(server));
    

    struct uav_data uav_data;

    while(!g->viewer->done())
    {
        recvfrom(s,&uav_data,sizeof(uav_data),0,&client,&client_len);

        updateCamera(g, uav_data.pitch, uav_data.roll);
        updatePosition(g, uav_data.NED, uav_data.q);
    }
    
    close(s);

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

/**
 * Free the memory from OSG
 */
void shutdown(struct global_struct *g)
{
    // TODO: Clean up weak pointers properly   
    free(g);
}

void help()
{
	cout << "This command line tool lets you visualize the uav "
	<< "flight using OpenSceneGraph." << endl
	<< "Usage:" "visualize [--earth] [--home] [--alt] [--fps] [--scale] [--start_frame] [--rot]" << endl
	<< " " << "--earth:    OSG Earth file"<< endl
	<< " " << "--home:     Latitude longitude altitude."<< endl
	<< " " << "--alt:      Initial vehicle altitude"<< endl;
}

/**
 * The matlab entry function
 */
int main(int argc, char * const argv[])
{
	
	if(argc==1){
		//Simple demo, load map of Boston
	}
	else{
		for (int i=1; i<argc; i++){
			if (strncmp("-", argv[i], 1)==0 ){
				if(strncmp("h", argv[i]+1, 1)==0 || strncmp("-help", argv[i]+1, 5)==0){
					//Help
					help();
					exit (1);
				}				
				else if(strncmp("-earth", argv[i]+1, 6)==0){
					earthFile=argv[i+1];
					useOSGEarth=true;
					i++;
				}
				else if(strncmp("-world", argv[i]+1, 6)==0){
					worldFile=argv[i+1];
					useOSGEarth=false;
					i++;
				}
				else if(strncmp("-model", argv[i]+1, 6)==0){
					modelFile=argv[i+1];
					i++;
				}
				else if(strncmp("-home", argv[i]+1, 5)==0){
					homeLat=atof(argv[i+1]);
					homeLon=atof(argv[i+2]);
					homeAlt=atof(argv[i+3]);
					i+=3;
				}
				else if(strncmp("-NED", argv[i]+1, 4)==0){
					initialNED[0]=atof(argv[i+1]);
					initialNED[1]=atof(argv[i+2]);
					initialNED[2]=atof(argv[i+3]);
				}
				else if(strncmp("-alt", argv[i]+1, 4)==0){
					initialAlt=atof(argv[i+1]);
					i++;
				}
			}
		}
	}

    // Start visualization in a new thread
    struct global_struct *g = initialize();

    //! Start a thread to get the network information
    startRunThread(g);

    //! Run the visualization
    g->viewer->run();

    //! Free the memory from the visualizatin
    shutdown(g);

    return 0;
}

