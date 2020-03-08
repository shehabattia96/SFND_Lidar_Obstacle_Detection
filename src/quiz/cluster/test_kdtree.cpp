// test functions for kdtree.h
#include "kdtree.h"
#include <assert.h> 


// At treeDepth 0, expected value is the first dimension (X = -6.2)
void test_getValueOfPointAtSpecifiedDepth_treeDepthZero(std::vector<float>* point, KdTree* tree) {
    unsigned int treeDepth = 0;
    float value = tree->getValueOfPointAtSpecifiedDepth(point, treeDepth);
    assert((*point)[0] == value);
    std::cout << "test1_treeDepthZero test passed" << std::endl;
}
// At treeDepth 1, expected value is the first dimension (Y = 7)
void test_getValueOfPointAtSpecifiedDepth_treeDepthOne(std::vector<float>* point, KdTree* tree) {
    unsigned int treeDepth = 1;
    float value = tree->getValueOfPointAtSpecifiedDepth(point, treeDepth);
    assert((*point)[1] == value);
    std::cout << "test1_treeDepthOne test passed" << std::endl;
}

void test_getValueOfPointAtSpecifiedDepth() {
    std::vector<float> point = {-6.2,7};
	unsigned int numDimensions = 2;
	KdTree* tree = new KdTree(numDimensions);
    test_getValueOfPointAtSpecifiedDepth_treeDepthZero(&point, tree);
    test_getValueOfPointAtSpecifiedDepth_treeDepthOne(&point, tree);
}

void test_transverseTreeToInsertPoint() {
    std::cout << "Starting test transverseTreeToInsertPoint." << std::endl;
	unsigned int numDimensions = 2;
	KdTree* tree = new KdTree(numDimensions);

    std::vector<std::vector<float>> points = { {0,0}, {0,1}, {-1,0}, {0,2} };
  
    for (int i=0; i<points.size(); i++) 
    	tree->insert(points[i],i); 
    std::cout << "Done creating tree. Starting assertions" << std::endl;
    std::cout << "Root point should be points[0]" << std::endl;
    assert(tree->root->point == points[0]);
    std::cout << "Root's first right should be points[1]" << std::endl;
    assert(tree->root->right->point == points[1]);
    std::cout << "To the right of root's first right should be points[3]" << std::endl;
    assert(tree->root->right->right->point == points[3]);
    std::cout << "Root's first left should be points[2]" << std::endl;
    assert(tree->root->left->point == points[2]);
}


void test_euclideanDistance() {
	KdTree* tree = new KdTree(NULL);
    std::vector<std::vector<float>> points2D = { {0,0}, {0,1}, {1,0} };
    assert(1 == tree->euclideanDistance(points2D[0], points2D[1]));
    assert(1 == tree->euclideanDistance(points2D[0], points2D[2]));

    std::vector<std::vector<float>> points3D = { {2,0,0}, {2,2,0}, {2,0,2} };
    assert(2 == tree->euclideanDistance(points3D[0], points3D[1]));
    assert(2 == tree->euclideanDistance(points3D[0], points3D[2]));
}


void test_searchForNodesWithinDistanceOfTarget(){
    std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3}, {7.2,6.1}, {8.0,5.3}, {7.2,7.1}, {0.2,-7.1}, {1.7,-6.9}, {-1.2,-7.2}, {2.2,-8.9} };

	unsigned int numDimensions = 2;
	KdTree* tree = new KdTree(numDimensions);
  
    for (int i=0; i<points.size(); i++) 
    	tree->insert(points[i],i); 

  	SearchObject nearbySearchObject = tree->search({-6,7},3.0);
    std::vector<int> nearby = nearbySearchObject.ids;
    std::vector<int> expectedNearby = {0,1,2,3};
    // assert ids are correct
    std::cout << "assert ids are correct" << std::endl;
  	for(int index = 0; index < nearby.size(); index += 1)
      assert(nearby[index] == expectedNearby[index]);

    // assert bounding points are correct:
    std::cout << "assert bounding points are correct" << std::endl;
    assert(nearbySearchObject.boundaryPoints[0][0].value == points[1][0]);
    assert(nearbySearchObject.boundaryPoints[0][1].value == points[2][0]);
    assert(nearbySearchObject.boundaryPoints[1][0].value == points[3][1]);
    assert(nearbySearchObject.boundaryPoints[1][1].value == points[1][1]);
}


int main ()
{
    std::cout << "Starting test getValueOfPointAtSpecifiedDepth." << std::endl;
    test_getValueOfPointAtSpecifiedDepth();
    std::cout << "Finieshed test getValueOfPointAtSpecifiedDepth." << std::endl;

    std::cout << "Starting test transverseTreeToInsertPoint." << std::endl;
    test_transverseTreeToInsertPoint();
    std::cout << "Finished test transverseTreeToInsertPoint." << std::endl;

    std::cout << "Starting test euclideanDistance." << std::endl;
    test_euclideanDistance();
    std::cout << "Finished test euclideanDistance." << std::endl;
    
    std::cout << "Starting test searchForNodesWithinDistanceOfTarget." << std::endl;
    test_searchForNodesWithinDistanceOfTarget();
    std::cout << "Finished test searchForNodesWithinDistanceOfTarget." << std::endl;
}