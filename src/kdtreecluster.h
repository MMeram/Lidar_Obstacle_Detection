#pragma once


#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>
#include <pcl/common/common.h>



// Node
struct Node
{
	std::vector<float> point;
	int level; // level
    int id; // unique identifier
	std::unique_ptr<Node> left;
	std::unique_ptr<Node> right;
	// Constructor
	Node(std::vector<float> arr, const uint8_t setLevel, const int sId );
};

// typedefs
typedef std::unique_ptr<Node> KDNode;
typedef std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI> > PclPointVector;
typedef std::vector<std::vector<int>> IndicesVector;
typedef std::vector<std::vector<float>> RawPointVector;

// KdTree
struct KdTree
{
	// root
	KDNode root;
	// dimension 
	uint8_t dim;
	// constructor
	KdTree(const uint8_t dimension);

	void insert(const std::vector<float>& point, const int id);

	// fill tree with the pcl type
	void fillTree(const PclPointVector& ptVector);

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(const std::vector<float>& target, const float distanceTol);

	void buildEuclideanCluster(IndicesVector& indicesVector, float distanceTol);
private:
	void proximity(const int idx, std::vector<bool> &memory,
				   std::vector<int> &cluster, float distanceTol);

private:
	RawPointVector mNodeVector;
};

namespace KDHelper
{
	// return the element for a particular level
	float getPointKey(const std::vector<float>& point, const uint8_t level);
	// get the key of Node
	float getNodeKey(const KDNode& node);

	// compare the elements, return 0, -1 or 1
	int compare(const std::vector<float>& point, const KDNode& node);
  

	// distance between a point and node
	float splitDistance(const std::vector<float>& point, const KDNode& node);

	// euclidean (L2) distance in 3D space
 	int distance(const std::vector<float>& point, const std::vector<float>& target);

	// if the node is within the box with the target on its center
	bool inBox(const std::vector<float>& target, const KDNode& node, float distanceTol);

	// balanced constructed tree
	// Points will be moved to the tree
    void balancedConstruct(std::unique_ptr<KdTree>& tree, std::vector<std::vector<float>>&& points);
	// standard insert method
	void insert(KDNode &node, const std::vector<float> &newPoint, const uint8_t level, const int id);
	
	// search algorithm w.r.t the rectangular with the target in its center
	void pointsInRange(std::vector<int>& points, const KDNode &node, const std::vector<float>& target, float distanceTol);

}






