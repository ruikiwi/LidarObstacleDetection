/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;		//x, y component
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}
	
	void insertInternal(Node*& parent, std::vector<float> point, int id, int depth) {
		if (!parent) {
			parent = new Node(point, id); 
			// std::cout << " add point" << std::endl; 
		} else {
			uint axisNum = depth % 2; 
			if (point[axisNum] < parent->point[axisNum]) {
				insertInternal(parent->left, point, id, depth+1);
				// std::cout << "traverse left" << std::endl; 

			} 
			else {
				insertInternal(parent->right, point, id, depth+1); 
				// std::cout << "traverse right" << std::endl; 
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		insertInternal(this->root, point, id, 0); 
	}

	bool isInBox(std::vector<float>& target, std::vector<float>& point, float distanceTol) {
		if ((point[0] < (target[0] - distanceTol)) || 
			(point[0] > (target[0] + distanceTol)) ||
			(point[1] < (target[1] - distanceTol)) ||
			(point[1] > (target[1] + distanceTol)))
			return false; 
		return true; 
	}

	float distance(std::vector<float>& target, std::vector<float>& point) {
		return sqrtf(pow(target[0] - point[0], 2.0) + pow(target[1] - point[1], 2.0)); 
	}


	void searchHelper(std::vector<float>& target, Node* node, int depth, float distanceTol, std::vector<int>& ids) {
		if (node) {
			if (isInBox(target, node->point, distanceTol)) {
				if (distance(target, node->point) < distanceTol) {
					ids.push_back(node->id); 
				}
			}
			uint axisNum = depth % 2; 
			if(target[axisNum] - distanceTol < node->point[axisNum]) {
				searchHelper(target, node->left, depth + 1, distanceTol, ids); 
			}
			if(target[axisNum] + distanceTol >= node->point[axisNum]) {
				searchHelper(target, node->right, depth + 1, distanceTol, ids); 
			}
		}
	}


	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids); 
		return ids;
	}
	
};




