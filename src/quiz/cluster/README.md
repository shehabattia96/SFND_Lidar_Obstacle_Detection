# KDTree implementation

Our KDTree is going to be a binary tree to represent 2D/3D points to make it easier to search for points.

## assumptions

- To insert a new node (newNode) into the Tree, we will compare the value of newNode from the dimension corresponding the the tree depth. For example, if our nodes are three dimensional (X, Y, Z), starting at depth = 0, we will transverse the tree. At depth 0 => we will compare newNode's X value to the X values of the nodes at depth 0. At depth 1 => we will compare newNode's Y value to the Y values of the nodes at depth 1, depth 2 => we will compare newNode's Z value to the Z values of the nodes at depth 2, depth 3 => we will compare X, depth 4 => Y, and so on..
  - Hence we will use `dimensionToCompare = treeDepth % numDimensions`, where numDimensions = 2 for (X, Y) and 3 for (X, Y, Z)

## future enhancements

- For some reason, the pointers were immutable. I had to reassign the left/right Node pointers after their places were recursively calculated. The immutable behavior doesn't make sense to me conceptually -> a pointer identified the location of a memory address, if I assign `new Node()` to that memory address, it should stick. It is likely that I have not done the implementation correctly.
- The kd-tree needs to be furthur optimized for accuracy. The way it's built now, if a point is larger than its parent node in one axis, it ends up on the right of the tree. This may not necessarily be correct for other dimensions of the point -- although this may be the point of this tree implementation, I wonder if it affects the performance of clustering later.

## Resources

- Did some reading on R-Trees vs Kd-Trees, looks like Kd-Trees are better for in-memory operations and smaller data sets. I really want some performance charts though.. Sources: https://blog.mapbox.com/a-dive-into-spatial-search-algorithms-ebd0c5e39d2a and https://stackoverflow.com/a/4345735/9824103