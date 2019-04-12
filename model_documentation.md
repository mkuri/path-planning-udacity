### Generating Paths

1. Caluculate the cost for each lanes

  * Get distance from the front vehicle of the lane
  * Base cost = exp(1/distance)
  * Check collision
    * Get distance from the behind vehicle of the lane
    * Get velocity of the behind vehicle of the lane
    * If the distance is small and the velocity is high, add the cost

2. Choose the lane according to the costs

3. If the distance to the car ahead is close, reduce the speed

4. Generate a trajectory based on the target lane and the target vehicle speed

  * Add the previous path to the target points
  * According to the target lane, add the target points 30m to 120m ahead
  * Map the target points to the spline space to be mathematically easy
  * Determine the movement distance per delta t from the target vehicle speed and target acceleration
  * Interpolate using spline 
  * Map the target points at the spline space to the xy space
