/* \author Aaron Brown */
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

struct KdTree
{
	Node* root;
	unsigned int numDimensions; // numDimensions = 2 for (X, Y) and 3 for (X, Y, Z)

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
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};




