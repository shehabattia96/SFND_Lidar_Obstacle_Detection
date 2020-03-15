# RANSAC implementation for a line and a plane

## implementation plan

### hyper-parameters

- distanceTolerance -> The distance at which a point will be considered an inlier
- maxIterations -> The number of random samples we'll pull and fit

### assumptions

- rand() returns a number > cloud.size()
- rand() returns pseudo-random integers for the random sampling to occur

### calculating distance of a point from a line

#### Approach 1: within two parallel lines approach

- Given two starting points (x1, y1) and (x2, y2).
- Two points define a line `y = mx + c`, where
  - `m = (y1 - y2) / (x1 - x2)`
  - `c = y1 - m * x1`
- Given a perpendicular distance of `d`, we will calculate two parallel lines for lower and upper bounds:
  - Given points x1 and y1 on the original line, we find the corresponding points the parallel line `L`.
    - `Lc = ((2 * sqrt(d^2 * m^2 + d^2 - m^2 * x1^2 + 2 * m * x1 * y1 - y1^2))/m + y1/m^2 - m * x1 + x1/m + 3 * y1)/(1/m^2 + 1)`, where Lc is the y-intercept at a distance `d` and slope `m` corresponding to `x1` and `y1` on the original point
    - Now we have lines `lineUpperBound = m * Lx + Lc` and `lineLowerBound = m * Lx + (-1 * Lc)`
- For each point, evaluate if `Px` is less than `lineUpperBound` and greater than `lineLowerBound`. If it is, then it's an inlier.

- After the fact, found out that the sqrt part evaluates to NaN because y value is too big. So I simplified the equation and assumed y=0 always. Worked perfectly:

```cpp
float xWhenYisZero = -1 * c / m;
float Lc = ((2 * std::sqrt((distanceTol*distanceTol) * (m*m) + (distanceTol*distanceTol) - (m*m) * (xWhenYisZero*xWhenYisZero)))/m - m * xWhenYisZero + xWhenYisZero/m)/(1/(m*m) + 1);
```

##### Proof for calculating `Lc` (this may be wrong for any use-cases I did not test. I tested only points in 1st and 2nd quadrants)

- `Ly = m * x1 + Lc` and `Lx = (y1 - Lc) / m`. Now we have points `(x1, Ly)` and `(Lx, y1)`.
- Midpoint `M` = (`(x1 - Lx)/2`, `(Ly - y1)/2`)
- `d = sqrt((x1 - Mx)^2 + (y1 - My)^2)`
- `d^2 = x1^2 -2*x1*Mx + Mx^2 + y1^2 - 2*y1*My + My^2`
- `d^2 = x1^2 -2*x1*(x1 - (y1 - Lc) / m)/2 + ((x1 - (y1 - Lc) / m)/2)^2 + y1^2 - 2*y1*(((m * x1 + Lc) - y1)/2) + (((m * x1 + Lc) - y1)/2)^2`
  -Worlfram Alpha friendly equation: `T = ((-2 sqrt(d^2 m^2 + d^2 - m^2 x1^2 + 2 m x1 y1 - y1^2))/m + y1/m^2 - m x1 + x1/m + 3 y1)/(1/m^2 + 1) where m=1 and x1=-1 and y1=0 and d=1`

#### Approach 2: distance from center approach (may be costly to perform perpendicular distance calculation for each point on a large dataset)

- Given two starting points (x1, y1) and (x2, y2).
- Two points define a line `Ax + By + C = 0`, where
  - `A = y1 - y2`
  - `B = x1 - x2`
  - `C = x1 * y2 - x2 * y1`
- The perpendicular distance of a point P from the line is `abs(A * Px + B * Py + C) / sqrt(A * A + B * B)`

##### Conclusion from comparing approach 1 and approach 2

- On a 2D dataset, approach 1 performed the line finding in (154, 178, 194 microseconds) and apprach 2 performed the line finding in (221, 220 and 184 microseconds). This is a small sample size and if I ran the test iteratively for longer, I may find that they perform very similarly because of the result from the 3D dataset.

- On a 3D dataset, approach 1 performed the line finding in (2765, 2713 microseconds) and apprach 2 performed the line finding in (2716, 2713 microseconds).

- Lastly, I would have had to hardcode two points verify each method's accuracy. I did not do this. But visually, they look like they're finding the optimal line well.

### calculating the distance of a point from a plane

- Given three starting points (x1, y1, z1), (x2, y2, z2) and (x3, y3, z3)
- Calculate two vectors `v1 = (x2 - x1, y2 - y1, z2 - z1)` and `v2 = (x3 - x1, y3 - y1, z3 - z1)`
- Calculate the cross product of V = v1 x v2. `Vx = (y2 − y1) * (z3 − z1) − (z2 − z1) * (y3 − y1)`. `Vy = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1)`. `Vz = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1)`
- Write the plane equation in standard form: Ax + By + Cz + D = 0 with reference to Point 1 (x1, y1, z1):
  - A, B, C = Vx, Vy and Vz respectively.
  - `D = -1 * (A * x1 + B * y1 + C * z1)`
- To calculate the distance between a point and a plane, use the D found in standard form:
  - `Distance =  fabs( A * x + B * y + C * z + D) / sqrt(A * A + B * B + C * C)`

#### Comparing the performance of the custom RANSAC algorithm with PCL's algorithm

- At first, the custom algorithm appeared to be twice slower than the PCL algorithm using hyper-parameters (MaxIter=10, disTol=0.5): On a 3D dataset, the custom algorith separated the plane in (15095, 14815 and 11406 microseconds). The PCL algorithm separate the plane in (9118, 7338 and 7424 microseconds).
  - However, after playing with the hyper-parameters, I noticed that the PCL algorithm stops iterating when it's found an optimal solution BEFORE hitting max iterations, hence cutting the computation time.
    - If I lower the hyper-parameters to (MaxIter=5, disTol=0.5), the custom algorithm also separates the plane in (7401, 6861 and 7394 micro-seconds)!
      - Optimizing hyper-parameters is key, but also finding the "good enough" solution may be a good stopping point for an iterative algorithm like the RANSAC.
