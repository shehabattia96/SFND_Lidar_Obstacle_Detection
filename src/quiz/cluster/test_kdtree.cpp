// test functions for kdtree.h
#include "kdtree.h"
#include <assert.h> 


// At treeDepth 0, expected value is the first dimension (X = -6.2)
void test_getValueOfPointAtSpecifiedDepth_treeDepthZero(std::vector<float>* point, KdTree* tree) {
    unsigned int treeDepth = 0;
    float value = tree->getValueOfPointAtSpecifiedDepth(point, treeDepth);
    assert ((*point)[0] == value);
    std::cout << "test1_treeDepthZero test passed" << std::endl;
}
// At treeDepth 1, expected value is the first dimension (Y = 7)
void test_getValueOfPointAtSpecifiedDepth_treeDepthOne(std::vector<float>* point, KdTree* tree) {
    unsigned int treeDepth = 1;
    float value = tree->getValueOfPointAtSpecifiedDepth(point, treeDepth);
    assert ((*point)[1] == value);
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


int main ()
{
    std::cout << "Starting test getValueOfPointAtSpecifiedDepth." << std::endl;
    test_getValueOfPointAtSpecifiedDepth();
    std::cout << "Finieshed test getValueOfPointAtSpecifiedDepth." << std::endl;
    std::cout << "Starting test transverseTreeToInsertPoint." << std::endl;
    test_transverseTreeToInsertPoint();
    std::cout << "Finished test transverseTreeToInsertPoint." << std::endl;
}