
#include "kdtreecluster.h"
#include <memory>


// Constructor Node
Node::Node(std::vector<float> arr, const uint8_t setLevel, const int sId )
	:	point(std::move(arr)), level(setLevel), id(sId), left(nullptr), right(nullptr)
	{}


// ---------------KdTree---------------

//Constructor
KdTree::KdTree(const uint8_t dimension)
	: root()
	, dim(dimension)
	, mNodeVector{{}}
	{}

// insert
void KdTree::insert(const std::vector<float>& point, const int id)
{
	KDHelper::insert(root, point, 0, id);
}

// insert point-cloud point
void KdTree::fillTree(const PclPointVector& pts)
{
	mNodeVector.clear();
	mNodeVector.reserve(pts.size());
	for (int i = 0; i < pts.size(); ++i)
    {
        mNodeVector.push_back({pts[i].x,pts[i].y,pts[i].z});
        insert(mNodeVector.back(), i);
    }
}

void KdTree::buildEuclideanCluster(IndicesVector &indicesVector, float distanceTol) 
{
	// we use a simple boolean vector/dynamic bitset  to track already clustered elements
	std::vector<bool> memory(mNodeVector.size(),true);
	for (int idx = 0; idx < mNodeVector.size(); ++idx)
	{
		if (memory[idx])
		{
			std::vector<int> clu;
			proximity(idx, memory, clu, distanceTol);
			indicesVector.emplace_back(clu);
		}
	}
}
// helper method
void KdTree::proximity(const int idx, std::vector<bool> &memory,
				   std::vector<int> &cluster, float distanceTol)
{
	if (memory[idx])
	{
		// mark it as clustered
		memory[idx] = false;
		cluster.push_back(idx);
		auto pts_nearby = search(mNodeVector[idx], distanceTol);
		for (auto &nearby_idx : pts_nearby)
		{
			proximity(nearby_idx, memory, cluster, distanceTol);
		}
	}
}


// search
std::vector<int> KdTree::search(const std::vector<float>& target, const float distanceTol)
{
    std::vector<int> points{};
    KDHelper::pointsInRange(points, root, target, distanceTol);
    return points;
}


namespace KDHelper
{
	// return the element for a particular level
	float getPointKey(const std::vector<float>& point, const uint8_t level)
	{
		auto elemIdx = level % point.size();
		return point[elemIdx];

	}

	// get the key of Node
	float getNodeKey(const KDNode& node)
	{
		return getPointKey(node->point, node->level);
	}

	// compare the elements, return 0, -1 or 1
	int compare(const std::vector<float>& point, const KDNode& node)
	{
		auto cmp = getPointKey(point, node->level) - getNodeKey(node);
		return 0 == cmp ? 0 : (std::signbit(cmp) ? -1 : +1);
	}
  

	// distance between a point and node
	float splitDistance(const std::vector<float>& point, const KDNode& node)
	{
		return std::fabs(getPointKey(point, node->level) - getNodeKey(node));
	}

	// euclidean (L3) distance in 3D space
 	int distance(const std::vector<float>& point, const std::vector<float>& target)
    {
        return std::sqrt(std::pow(point[0] - target[0],2) + std::pow(point[1] - target[1],2)
			+ std::pow(point[2] - target[2],2));
    }

	// if the node is within the box in 3D space with the target on its center
	bool inBox(const std::vector<float>& target, const KDNode& node, float distanceTol)
    {
		return (std::fabs (node->point[0] - target[0]) <= distanceTol) && 
				(std::fabs (node->point[1] - target[1]) <= distanceTol) && 
				(std::fabs (node->point[2] - target[2]) <= distanceTol) ;
    }

	// balanced constructed tree
	// Points will be moved to the tree
    void balancedConstruct(std::unique_ptr<KdTree>& tree, std::vector<std::vector<float>>&& points)
    {
        int i = 0;
        while( points.size() > 0 ) 
        {
			// pick up the elem w.r.t. the level
            auto crit = i % tree->dim == 0 ? 0 : 1;
			// sort against it
            std::sort(points.begin(), points.end(), [&]( auto const& point1, auto const& point2){
                return point1[crit]<point2[crit];
            });
			// pick up the median
            auto itr = points.begin() + points.size()/2; 
			// insert
            tree->insert(*itr,i);
			// erase from the original point vector
            points.erase(itr); 
            ++i;
        }
    }


	// standard insert method
	void insert(KDNode &node, const std::vector<float> &newPoint, const uint8_t level, const int id)
	{
		if (!node)
		{
			node = std::make_unique<Node>(newPoint, level, id);
		}
		else if (KDHelper::compare(newPoint, node) < 0)
		{
			// insert to the left
			insert(node->left, newPoint, node->level + 1, id);
		}
		else 
		{
			// insert to the right
			insert(node->right, newPoint, node->level + 1, id);
		}
	}
	
	// search algorithm w.r.t the rectangular with the target in its center
	void pointsInRange(std::vector<int>& points, const KDNode &node, const std::vector<float>& target, float distanceTol)
	{
		if (node)
		{
			if (inBox(target, node, distanceTol))
			{
				// in the box, so calculate L3 distance
				if (distance(node->point, target) < distanceTol)
				{
					points.push_back(node->id);
				}
			}
			//in a k-d tree, at each level i we only compare the i-th (modulo k) coordinate of points,
			// to decide which branch of the tree will be traversed.
			if (getPointKey(target, node->level) - distanceTol <= getNodeKey(node))
			{
				// left branch traversal
				pointsInRange(points, node->left, target, distanceTol);
			}
			if (getPointKey(target, node->level) + distanceTol >= getNodeKey(node))
			{
				// right branch traversal
				pointsInRange(points, node->right, target, distanceTol);
			}
		}
	}
}







