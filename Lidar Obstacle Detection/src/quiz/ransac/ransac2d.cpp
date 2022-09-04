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

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
    while (maxIterations--)
    {
        int index1, index2;
        index1 = rand()%(cloud->points.size());
        index2 = (index1+(rand()%((cloud->points.size())-1))+1)%(cloud->points.size());
        
        std::unordered_set<int> inliers;
        inliers.insert(index1);
        inliers.insert(index2);
      
        float x1, y1, x2, y2, a, b, c;
      
        x1 = cloud->points[index1].x;
        y1 = cloud->points[index1].y;
      
        x2 = cloud->points[index2].x;
        y2 = cloud->points[index2].y;
      
        a = y1 - y2;
        b = x2 - x1;
        c = x1*y2 - x2*y1;
      
        for (int index=0; index<(cloud->points.size()); index++)
        {
            if ((index==index1) || (index==index2))
                continue;
            
            float x3, y3, d;
            x3 = cloud->points[index].x;
            y3 = cloud->points[index].y;
            
            d = fabs(a*x3 + b*y3 + c) / sqrt(a*a + b*b);
          
            if (d <= distanceTol)
                inliers.insert(index);
        }
      
        if (inliers.size() > inliersResult.size())
           inliersResult = inliers;
    }
  
	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
    while (maxIterations--)
    {
        int index1, index2, index3;
        index1 = rand()%(cloud->points.size());
        index2 = (index1+(rand()%((cloud->points.size())-1))+1)%(cloud->points.size());
        if (index1 > index2)
            if ((index1-index2+1) < (cloud->points.size()))
                index3 = (index1+(rand()%((cloud->points.size())-(index1-index2+1)))+1)%(cloud->points.size());
            else
                index3 = (index2+(rand()%((cloud->points.size())-2))+1)%(cloud->points.size());
        else
            if ((index2-index1+1) < (cloud->points.size()))
                index3 = (index2+(rand()%((cloud->points.size())-(index2-index1+1)))+1)%(cloud->points.size());
            else
                index3 = (index1+(rand()%((cloud->points.size())-2))+1)%(cloud->points.size());
        
        std::unordered_set<int> inliers;
        inliers.insert(index1);
        inliers.insert(index2);
        inliers.insert(index3);
      
        float x1, y1, z1, x2, y2, z2, x3, y3, z3, v1[3], v2[3], v3[3];
      
        x1 = cloud->points[index1].x;
        y1 = cloud->points[index1].y;
        z1 = cloud->points[index1].z;
      
        x2 = cloud->points[index2].x;
        y2 = cloud->points[index2].y;
        z2 = cloud->points[index2].z;
      
        x3 = cloud->points[index3].x;
        y3 = cloud->points[index3].y;
        z3 = cloud->points[index3].z;
      
        v1[0] = x2 - x1;
        v1[1] = y2 - y1;
        v1[2] = z2 - z1;
        
        v2[0] = x3 - x1;
        v2[1] = y3 - y1;
        v2[2] = z3 - z1;
      
        v3[0] = v1[1]*v2[2] - v1[2]*v2[1];
        v3[1] = v1[0]*v2[2] - v1[2]*v2[0];
        v3[2] = v1[0]*v2[1] - v1[1]*v2[0];
      
        for (int index=0; index<(cloud->points.size()); index++)
        {
            if ((index==index1) || (index==index2) || (index==index3))
                continue;
            
            float x4, y4, z4, d;
            x4 = cloud->points[index].x;
            y4 = cloud->points[index].y;
            z4 = cloud->points[index].z;
            
            d = fabs(v3[0]*x4 + v3[1]*y4 + v3[2]*z4 - v3[0]*x1 - v3[1]*y1 - v3[2]*z1) / sqrt(v3[0]*v3[0] + v3[1]*v3[1] + v3[2]*v3[2]);
          
            if (d <= distanceTol)
                inliers.insert(index);
        }
      
        if (inliers.size() > inliersResult.size())
           inliersResult = inliers;
    }
  
	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 50, 0.5);

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
