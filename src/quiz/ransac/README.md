# RANSAC implementation for a line and a plane

## implementation plan

### hyper parameters

- distanceTolerance -> The distance at which a point will be considered an inlier
- maxIterations -> The number of random samples we'll pull and fit

### assumptions

- rand() returns a number > cloud.size()

### calculating distance of a point from a line

#### within two parallel lines approach

- Find the min and max points in the dataset
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

#### distance from center approach (may be costly to perform perpendicular distance calculation for each point on a large dataset)

- Two points define a line `Ax + By + C = 0`, where
  - `A = y1 - y2`
  - `B = x1- x2`
  - `C = x1 * y2 - x2 * y1`
- The perpendicular distance of a point P from the line is `abs(A * Px + B * Py + C) / sqrt(A * A + B * B)`
