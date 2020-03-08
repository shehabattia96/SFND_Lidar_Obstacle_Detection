/* \author Aaron Brown */
// Modified by Shehab Attia 3/7/2020
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct BoundaryPoint
{
	int id;
	float value;
	Node* node;
	BoundaryPoint() : id(NULL), value(NULL), node(NULL) {}
	BoundaryPoint(int setId, float setValue, Node* setNode) : id(setId), value(setValue), node(setNode) {}
};

struct SearchObject
{
	std::vector<std::vector<BoundaryPoint>> boundaryPoints;
	std::vector<int> ids;
	SearchObject() {}
};

struct KdTree
{
	Node* root;
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

	Node* transverseTreeToInsertPoint(unsigned int treeDepth, std::vector<float> point, int id, Node* currentNode) {
		// assign first node
		if (currentNode == (struct Node *) NULL) {
			Node* newNode = new Node(point, id);
			currentNode = newNode;
			return currentNode;
		}
		// Find out which dimension we will compare
		unsigned int dimensionToCompare = treeDepth % this->numDimensions;

		float pointValue = this->getValueOfPointAtSpecifiedDepth(&point, dimensionToCompare);
		float currentNodeValue = this->getValueOfPointAtSpecifiedDepth(&(currentNode->point), dimensionToCompare);

		if (pointValue < currentNodeValue) {
			Node* leftNode = currentNode->left;
			currentNode->left = transverseTreeToInsertPoint(treeDepth + 1, point, id, leftNode);
			return currentNode;
		}
		
		Node* rightNode = currentNode->right;
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

	SearchObject* searchForNodesWithinDistanceOfTarget(Node* currentNode, SearchObject* searchObject, unsigned int treeDepth, std::vector<float> target, float distanceTol) {
		if (currentNode == (struct Node *) NULL) {
			return searchObject;
		}
		// Find out which dimension we will compare
		unsigned int dimensionToCompare = treeDepth % this->numDimensions;

		float targetValue = this->getValueOfPointAtSpecifiedDepth(&target, dimensionToCompare);
		std::vector<float> currentNodePoint = currentNode->point;

		// we'll go straight to calculating euclidean distance
		if (this->euclideanDistance(target, currentNodePoint) < distanceTol) {
			searchObject->ids.push_back(currentNode->id);
				
			for (int axis = 0; axis < this->numDimensions; axis += 1) {
				std::vector<BoundaryPoint> axisBoundaryPoints = searchObject->boundaryPoints[axis];
				BoundaryPoint* minAxisBoundaryPoint = &axisBoundaryPoints[0];
				BoundaryPoint* maxAxisBoundaryPoint = &axisBoundaryPoints[1];
				float currentNodeValueAtAxis = this->getValueOfPointAtSpecifiedDepth(&currentNodePoint, axis);
				if (minAxisBoundaryPoint->value == NULL || currentNodeValueAtAxis < minAxisBoundaryPoint->value) {
					minAxisBoundaryPoint->value = currentNodeValueAtAxis;
					minAxisBoundaryPoint->id = currentNode->id;
					minAxisBoundaryPoint->node = currentNode;
					searchObject->boundaryPoints[axis][0] = *minAxisBoundaryPoint;
				}
				
				if (maxAxisBoundaryPoint->value == NULL || currentNodeValueAtAxis > maxAxisBoundaryPoint->value) {
					maxAxisBoundaryPoint->value = currentNodeValueAtAxis;
					maxAxisBoundaryPoint->id = currentNode->id;
					maxAxisBoundaryPoint->node = currentNode;
					searchObject->boundaryPoints[axis][1] = *maxAxisBoundaryPoint;
				}
			}
		}

		float currentNodeValue = this->getValueOfPointAtSpecifiedDepth(&(currentNodePoint), dimensionToCompare);

		if (targetValue - distanceTol < currentNodeValue) {
			Node* leftNode = currentNode->left;
			searchObject = searchForNodesWithinDistanceOfTarget(leftNode, searchObject, treeDepth + 1, target, distanceTol);
		}

		if (targetValue + distanceTol > currentNodeValue) {
			Node* rightNode = currentNode->right;
			searchObject = searchForNodesWithinDistanceOfTarget(rightNode, searchObject, treeDepth + 1, target, distanceTol);
		}
		return searchObject;
	}

	// return a list of point ids in the tree that are within distance of target
	SearchObject search(std::vector<float> target, float distanceTol)
	{
		SearchObject* searchObject = new SearchObject();
		for (int index = 0; index < this->numDimensions; index += 1) {
			std::vector<BoundaryPoint> axisBoundaryPoints= { BoundaryPoint(), BoundaryPoint() };
			searchObject->boundaryPoints.push_back(axisBoundaryPoints);
		}
		searchObject = searchForNodesWithinDistanceOfTarget(root, searchObject, 0, target, distanceTol);
		return *searchObject;
	}

};




