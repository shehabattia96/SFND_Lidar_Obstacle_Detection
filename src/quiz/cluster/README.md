# KDTree implementation

Our KDTree is going to be a binary tree to represent 2D/3D points to make it easier to search for points.

## implementation plan

### inserting new nodes into Kd-Tree

- To insert a new node (newNode) into the Tree, we will compare the value of newNode from the dimension corresponding the the tree depth. For example, if our nodes are three dimensional (X, Y, Z), starting at depth = 0, we will transverse the tree. At depth 0 => we will compare newNode's X value to the X values of the nodes at depth 0. At depth 1 => we will compare newNode's Y value to the Y values of the nodes at depth 1, depth 2 => we will compare newNode's Z value to the Z values of the nodes at depth 2, depth 3 => we will compare X, depth 4 => Y, and so on..
  - Hence we will use `dimensionToCompare = treeDepth % numDimensions`, where numDimensions = 2 for (X, Y) and 3 for (X, Y, Z)

### searching Kd-Tree

- To search the Kd-Tree for interesting points, we'll split the problem into three parts
  - transversing the tree to find closest points, again using the same logic for comparison as inserting (`dimensionToCompare = treeDepth % numDimensions`).
  - ~~calculating bounding box between a point A, a known distance.~~ we're doing euclidean distance right away because I would assume that performance improvement is very small. I should do a small test to make sure though.
  - calculating eucladean distance to points of interest.

### Knn Clustering

- The solution in the lesson sounds pretty expensive. We're transversing the Kd-Tree, which is O(n) worst case, then for each unprocessed point, we're transversing again to get a list of nearby points (let's say, length x), so total time complexity = O(N^2+x).
  - If we can capture ~~max and min points in each direction while searching, we could assume all points in the ROI between min and max are processed, then do Kd-Tree search on just the boundary points.~~ (in hind-sight, this wasn't necessary since Kd-tree search handles iteratively finding closest neighbors.. HOWEVER, adding all the found IDs to the searchedNodes list was good.)
    - Then we BFS the entire tree and keep clustering

## future enhancements

- I was having issues when adding a new point to the KD-Tree with the Node pointers being immutable when dereferenced. I'm not sure if Struct pointers are immutable when being dereferenced but I had to reassign the left/right Node pointers after their places were recursively calculated. The immutable behavior doesn't make sense to me conceptually -> a pointer identified the location of a memory address, if I assign `new Node()` to that memory address, it should stick. It is likely that I have not done the implementation correctly.
- The kd-tree needs to be furthur optimized for accuracy. The way it's built now, if a point is larger than its parent node in one axis, it ends up on the right of the tree. This may not necessarily be correct for other dimensions of the point -- although this may be the point of this tree implementation, I wonder if it affects the performance of clustering later.
  - Edit: upon further research, my line of thought was correct and there is a way to "look back" on the adjacent node in the tree to make sure it's not a better answer to the closest point question. Source: https://youtu.be/DlPrTGbO19E?t=260
- Test to compare the performance of doing the bounding box calculation during closest neighbor searching vs doing euclidean distance right away.

## Resources

- Did some reading on R-Trees vs Kd-Trees, looks like Kd-Trees are better for in-memory operations and smaller data sets. I really want some performance charts though.. Sources: https://blog.mapbox.com/a-dive-into-spatial-search-algorithms-ebd0c5e39d2a and https://stackoverflow.com/a/4345735/9824103