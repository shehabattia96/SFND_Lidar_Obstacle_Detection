/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac2DLineUsingBounds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	auto startTime = std::chrono::steady_clock::now();
	std::unordered_set<int> inliersResult;
	srand(time(NULL)); // initialize random number generator

	for (int iteration = 0; iteration < maxIterations; iteration += 1) {

		std::unordered_set<int> inliers;
		while (inliers.size() < 2) {
			inliers.insert(rand()%(cloud->points.size()));
		}

		float x1, x2, y1, y2;
		auto inliersIterator = inliers.begin();
		x1 = cloud->points[*inliersIterator].x;
		y1 = cloud->points[*inliersIterator].y;
		inliersIterator++;
		x2 = cloud->points[*inliersIterator].x;
		y2 = cloud->points[*inliersIterator].y;

		float m = (y1 - y2) / (x1 - x2);
		float c = y1 - m * x1;
		// Lc calculation proof is in the README file
		// float Lc = ((2 * std::sqrt((distanceTol*distanceTol) * (m*m) + (distanceTol*distanceTol) - (m*m) * (x1*x1) + 2 * m * x1 * y1 - (y1*y1)))/m + y1/(m*m) - m * x1 + x1/m + 3 * y1)/(1/(m*m) + 1);
		float xWhenYisZero = -1 * c / m;
		float Lc = ((2 * std::sqrt((distanceTol*distanceTol) * (m*m) + (distanceTol*distanceTol) - (m*m) * (xWhenYisZero*xWhenYisZero)))/m - m * xWhenYisZero + xWhenYisZero/m)/(1/(m*m) + 1);

		auto lineUpperBound = [](float Lx, float m, float Lc) {return m * Lx + Lc;};
		auto lineLowerBound = [](float Lx, float m, float Lc) {return m * Lx + (-1 * Lc);};

		for (int index = 0; index < cloud->points.size(); index += 1) {
			if (inliers.count(index) > 0) continue;
			pcl::PointXYZ point = cloud->points[index];
			float pointX = point.x;
			float pointY = point.y;
			float lineUpperBoundY = lineUpperBound(pointX, m, Lc);
			float lineLowerBoundY = lineLowerBound(pointX, m, Lc);
			if (pointY <= lineUpperBoundY && pointY >= lineLowerBoundY)
				inliers.insert(index);
		}

		if (inliers.size() > inliersResult.size()) {
			inliersResult = inliers;
		}
	}
	
	
	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
	std::cout << "Line RANSAC elapsedTime: " << elapsedTime.count() << " micro-seconds." << std::endl;
	
	return inliersResult;

}

std::unordered_set<int> Ransac2DLineUsingNormalDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	auto startTime = std::chrono::steady_clock::now();
	std::unordered_set<int> inliersResult;
	srand(time(NULL)); // initialize random number generator

	for (int iteration = 0; iteration < maxIterations; iteration += 1) {

		std::unordered_set<int> inliers;
		while (inliers.size() < 2) {
			inliers.insert(rand()%(cloud->points.size()));
		}

		float x1, x2, y1, y2;
		auto inliersIterator = inliers.begin();
		x1 = cloud->points[*inliersIterator].x;
		y1 = cloud->points[*inliersIterator].y;
		inliersIterator++;
		x2 = cloud->points[*inliersIterator].x;
		y2 = cloud->points[*inliersIterator].y;

		float a = (y1 - y2);
		float b = (x2 - x1);
		float c = (x1 * y2 - x2 * y1);

		for (int index = 0; index < cloud->points.size(); index += 1) {
			if (inliers.count(index) > 0) continue;
			pcl::PointXYZ point = cloud->points[index];
			float pointX = point.x;
			float pointY = point.y;
			float distance = fabs(a * pointX + b * pointY + c) / sqrt(a * a + b * b);
			if (distance <= distanceTol)
				inliers.insert(index);
		}

		if (inliers.size() > inliersResult.size()) {
			inliersResult = inliers;
		}
	}
	
	
	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
	std::cout << "Line RANSAC elapsedTime: " << elapsedTime.count() << " micro-seconds." << std::endl;
	
	return inliersResult;
}

std::unordered_set<int> Ransac3DLineUsingNormalDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	auto startTime = std::chrono::steady_clock::now();
	std::unordered_set<int> inliersResult;
	srand(time(NULL)); // initialize random number generator

	for (int iteration = 0; iteration < maxIterations; iteration += 1) {

		std::unordered_set<int> inliers;
		while (inliers.size() < 3) {
			inliers.insert(rand()%(cloud->points.size()));
		}

		float x1, x2, x3, y1, y2, y3, z1, z2, z3;
		auto inliersIterator = inliers.begin();
		x1 = cloud->points[*inliersIterator].x;
		y1 = cloud->points[*inliersIterator].y;
		z1 = cloud->points[*inliersIterator].z;
		inliersIterator++;
		x2 = cloud->points[*inliersIterator].x;
		y2 = cloud->points[*inliersIterator].y;
		z2 = cloud->points[*inliersIterator].z;
		inliersIterator++;
		x3 = cloud->points[*inliersIterator].x;
		y3 = cloud->points[*inliersIterator].y;
		z3 = cloud->points[*inliersIterator].z;

		// explanation for calculating these coefficients is in the README.md
		float A = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
		float B = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
		float C = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
		float D = -1 * (A * x1 + B * y1 + C * z1);

		for (int index = 0; index < cloud->points.size(); index += 1) {
			if (inliers.count(index) > 0) continue;
			pcl::PointXYZ point = cloud->points[index];
			float pointX = point.x;
			float pointY = point.y;
			float pointZ = point.z;

			float distance =  fabs( A * pointX + B * pointY + C * pointZ + D) / sqrt(A * A + B * B + C * C);
			
			if (distance <= distanceTol)
				inliers.insert(index);
		}

		if (inliers.size() > inliersResult.size()) {
			inliersResult = inliers;
		}
	}
	
	
	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
    std::cout << "Found " << inliersResult.size() << " inliers" << std::endl;
	std::cout << "Line RANSAC elapsedTime: " << elapsedTime.count() << " micro-seconds." << std::endl;
	
	return inliersResult;
}

std::unordered_set<int> Ransac3DLineUsingPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	auto startTime = std::chrono::steady_clock::now();
	srand(time(NULL)); // initialize random number generator

	 // references http://pointclouds.org/documentation/tutorials/extract_indices.php#extract-indices
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceTol);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    std::cout << "Found " << inliers->indices.size() << " inliers" << std::endl;
	
	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
	std::cout << "Line RANSAC elapsedTime: " << elapsedTime.count() << " micro-seconds." << std::endl;
	
	std::unordered_set<int> inliersResult(inliers->indices.begin(), inliers->indices.end());
	return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	


	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	// std::unordered_set<int> inliers = Ransac2DLineUsingNormalDistance(cloud, 10, 0.5);
	// std::unordered_set<int> inliers = Ransac2DLineUsingBounds(cloud, 10, 0.5);
	std::unordered_set<int> inliers = Ransac3DLineUsingNormalDistance(cloud, 5, 0.5);
	// std::unordered_set<int> inliers = Ransac3DLineUsingPCL(cloud, 5, 0.5);
	

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
