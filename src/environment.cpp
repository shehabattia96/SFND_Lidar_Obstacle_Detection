/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors
#include <filesystem>

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    bool renderLidarRays = false;
    bool renderLidarPointCloud = false;
    bool renderSegmentedGround = true;
    bool renderSegmentedEntities = true;
    bool renderClusteredEntities = true;

    // render initial scene
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    double groundSlope = 0.0;
    Lidar* lidar = new Lidar(cars, groundSlope); 

    pcl::PointCloud<pcl::PointXYZ>::Ptr lidarCloud = lidar->scan();

    // display lidarCloud in viewer
    if (renderLidarRays) renderRays(viewer, lidar->position, lidarCloud);
    if (renderLidarPointCloud) renderPointCloud(viewer, lidarCloud, "lidarCloud");

    // segment ground from scene entities using RANSAC
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    int maxIterations (10);
    float distanceThreshold (0.2f);
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(lidarCloud, maxIterations, distanceThreshold);
    if (renderSegmentedGround) renderPointCloud(viewer,segmentCloud.first, "groundPcd", Color(1,1,1));
    if (renderSegmentedEntities) renderPointCloud(viewer,segmentCloud.second, "entitiesPcd", Color(1,1,0)); 

    // cluster scene entities
    float clusterTolerance (1.0);
    int minSize (3);
    int maxSize (30);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.second, clusterTolerance, minSize, maxSize);

    // render clusters
    if (renderClusteredEntities) {
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
        for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
        {
            std::cout << "cluster size "; pointProcessor.numPoints(cluster);
            renderPointCloud(viewer, cluster, "entitiesCluster"+std::to_string(clusterId), colors[clusterId]);

            Box box = pointProcessor.BoundingBox(cluster);
            renderBox(viewer, box, clusterId);

            ++clusterId;
        }
    }
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud) {
    // render options
    bool renderRawPcd = false;
    bool downsampledPcd = false;
    bool renderSegmentedGround = false;
    bool renderSegmentedEntities = true;
    bool renderClusteredEntities = true;

    // car
    Box egoCar;
    egoCar.x_min = -1.5;
    egoCar.y_min = -1.6;
    egoCar.z_min = -2;
    egoCar.x_max = 2.6;
    egoCar.y_max = 1.6;
    egoCar.z_max = -0.4;
    Eigen::Vector4f egoCarMinPoint (egoCar.x_min, egoCar.y_min, egoCar.z_min, 1);
    Eigen::Vector4f egoCarMaxPoint (egoCar.x_max, egoCar.y_max, egoCar.z_max, 1);

    if (renderRawPcd) renderPointCloud(viewer, inputCloud, "inputCloud");

    // downsample the point cloud
    float filterRes (0.2);
    struct WorldSize { float x = 15.f; float y = 7.f; float z = 10.f; } worldSize;
    Eigen::Vector4f minPoint (-worldSize.x, -worldSize.y, egoCar.z_min,1);
    Eigen::Vector4f maxPoint (worldSize.x, worldSize.y, worldSize.z,1);

    // downsample by filterRes and crop by worldSize
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, filterRes, minPoint, maxPoint);
    // remove any points inside the car's bounding box
    filterCloud = pointProcessorI->CropCloud(filterCloud, egoCarMinPoint, egoCarMaxPoint, true);
    if (downsampledPcd) renderPointCloud(viewer, filterCloud, "filterCloud");
    renderBox(viewer, egoCar, -1);

    // segment ground from scene entities using RANSAC
    ProcessPointClouds<pcl::PointXYZI> pointProcessor;
    int maxIterations (10);
    float distanceThreshold (0.2f);
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor.SegmentPlane(filterCloud, maxIterations, distanceThreshold);
    if (renderSegmentedGround) renderPointCloud(viewer,segmentCloud.first, "groundPcd", Color(1,1,1));
    if (renderSegmentedEntities) renderPointCloud(viewer,segmentCloud.second, "entitiesPcd", Color(1,1,0)); 

    // cluster scene entities
    float clusterTolerance (0.5);
    int minSize (10);
    int maxSize (500);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.second, clusterTolerance, minSize, maxSize);

    // render clusters
    if (renderClusteredEntities) {
    int clusterId = 0;
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size "; pointProcessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "entitiesCluster"+std::to_string(clusterId), Color());

        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }
    }
}

void loopFrame(typename pcl::visualization::PCLVisualizer::Ptr viewer) {
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    }
}

void loopFrame(typename pcl::visualization::PCLVisualizer::Ptr viewer, typename pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI, std::vector<boost::filesystem::path>* stream, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI) {
    auto streamIterator = stream->begin();
    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream->end())
            streamIterator = stream->begin();

        viewer->spinOnce ();
    }
};

enum EnvironmentEnum { simpleHighwayEnv, cityBlockFileEnv, cityBlockStreamEnv };
std::map<std::string, EnvironmentEnum> EnvironmentMap = {
    {"simpleHighway", simpleHighwayEnv},
    {"cityBlockFile", cityBlockFileEnv},
    {"cityBlockStream", cityBlockStreamEnv},
};
namespace filesystem = std::experimental::filesystem;

int main (int argc, char** argv)
{
    // set environment from args
    EnvironmentEnum environment (simpleHighwayEnv);
    if (argc > 1) {
        std::string requestedEnvironment = argv[1];
        if (EnvironmentMap.count(requestedEnvironment) > 0) {
            environment = EnvironmentMap.at(requestedEnvironment);
        } else {
            std::cout << requestedEnvironment << " is not an environment that can be loaded.";
            exit(1);
        }
    }
    // define file paths
    filesystem::path currentPath = filesystem::current_path(); // this assumes you're running from root dir
    filesystem::path relativeDataFolder = "/src/sensors/data/pcd/data_1";
    filesystem::path dataFolder = currentPath;
    dataFolder += relativeDataFolder;
    filesystem::path pcdFile = dataFolder;
    pcdFile += "/0000000000.pcd";

    std::cout << "starting enviroment in " << currentPath << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    
    if (environment == simpleHighwayEnv) {
        simpleHighway(viewer);
        loopFrame(viewer);
    }

    if (environment == cityBlockStreamEnv || environment == cityBlockFileEnv) {
        ProcessPointClouds<pcl::PointXYZI>* pointProcessorI (new ProcessPointClouds<pcl::PointXYZI>);
        
        if (environment == cityBlockFileEnv) {
            if (!filesystem::is_regular_file(pcdFile)) {
                std::cout << "Couldn't read file " << pcdFile;
                exit(1);
            }
            pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd(pcdFile.u8string());
            std::cout << "input cloud loaded with number of points equal "; pointProcessorI->numPoints(inputCloud);
            cityBlock(viewer, pointProcessorI, inputCloud);
            loopFrame(viewer);
        }

        if (environment == cityBlockStreamEnv) {
            if (!filesystem::is_directory(dataFolder)) {
                std::cout << "Couldn't read folder " << dataFolder;
                exit(1);
            }
            std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd(dataFolder.u8string());
            pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
            loopFrame(viewer, inputCloudI, &stream, pointProcessorI);
        }
    }
}