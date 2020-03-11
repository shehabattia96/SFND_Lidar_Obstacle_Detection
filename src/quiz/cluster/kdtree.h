/* \author Aaron Brown */
// Modified by Shehab Attia 3/7/2020
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	std::shared_ptr<Node> left;
	std::shared_ptr<Node> right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	std::shared_ptr<Node> root;
	unsigned int numDimensions; // numDimensions = 2 for (X, Y) and 3 for (X, Y, Z)
	size_t treeLength = 0; // this will need to be updated if we introduce a remove function

	KdTree(unsigned int numDimensions)
	: root(NULL), numDimensions(numDimensions)
	{}

	float getValueOfPointAtSpecifiedDepth(std::vector<float>* point, unsigned int treeDepth) {
		if (point->empty()) {
			return NULL;
		}
		unsigned int dimensionToCompare = treeDepth % this->numDimensions;
		return (*point)[dimensionToCompare];
	}

	std::shared_ptr<Node> transverseTreeToInsertPoint(unsigned int treeDepth, std::vector<float> point, int id, std::shared_ptr<Node> currentNode) {
		// assign first node
		if (currentNode == NULL) {
			std::shared_ptr<Node> newNode(new Node(point, id));
			currentNode = newNode;
			return currentNode;
		}
		// Find out which dimension we will compare
		unsigned int dimensionToCompare = treeDepth % this->numDimensions;

		float pointValue = this->getValueOfPointAtSpecifiedDepth(&point, dimensionToCompare);
		float currentNodeValue = this->getValueOfPointAtSpecifiedDepth(&(currentNode->point), dimensionToCompare);

		if (pointValue < currentNodeValue) {
			std::shared_ptr<Node> leftNode = currentNode->left;
			currentNode->left = transverseTreeToInsertPoint(treeDepth + 1, point, id, leftNode);
			return currentNode;
		}
		
		std::shared_ptr<Node> rightNode = currentNode->right;
		currentNode->right = transverseTreeToInsertPoint(treeDepth + 1, point, id, rightNode);
		return currentNode;
	}

	void insert(std::vector<float> point, int id)
	{
		// We start searching the tree from depth 0
		root = transverseTreeToInsertPoint(0, point, id, root);
		treeLength += 1; // this will need to be updated if we introduce a remove function
	}

	// calculate euclidean distance for any size vector
	float euclideanDistance(std::vector<float> pointA, std::vector<float> pointB) {
		if (pointA.empty() || pointB.empty()) {
			return 0;
		}
		float distance;
		float sum = 0;
		for (int index = 0; index < pointA.size(); index += 1 ) {
			sum += ( (pointA[index] - pointB[index]) * (pointA[index] - pointB[index]) );
		}
		distance = sqrt(sum);
		return distance;
	}

	std::vector<int>* searchForNodesWithinDistanceOfTarget(std::shared_ptr<Node> currentNode, std::vector<int>* ids, unsigned int treeDepth, std::vector<float> target, float distanceTol) {
		if (currentNode == NULL) {
			return ids;
		}
		// Find out which dimension we will compare
		unsigned int dimensionToCompare = treeDepth % this->numDimensions;

		float targetValue = this->getValueOfPointAtSpecifiedDepth(&target, dimensionToCompare);
		std::vector<float> currentNodePoint = currentNode->point;

		// we'll go straight to calculating euclidean distance
		if (this->euclideanDistance(target, currentNodePoint) < distanceTol) {
			ids->push_back(currentNode->id);
		}

		float currentNodeValue = this->getValueOfPointAtSpecifiedDepth(&(currentNodePoint), dimensionToCompare);

		if (targetValue - distanceTol < currentNodeValue) {
			std::shared_ptr<Node> leftNode = currentNode->left;
			ids = searchForNodesWithinDistanceOfTarget(leftNode, ids, treeDepth + 1, target, distanceTol);
		}

		if (targetValue + distanceTol > currentNodeValue) {
			std::shared_ptr<Node> rightNode = currentNode->right;
			ids = searchForNodesWithinDistanceOfTarget(rightNode, ids, treeDepth + 1, target, distanceTol);
		}
		return ids;
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		ids = *searchForNodesWithinDistanceOfTarget(root, &ids, 0, target, distanceTol);
		return ids;
	}

};




