### Generating Paths

1. Caluculate the cost for each lanes
  1. Get distance from the front vehicle of the lane
  2. Base cost = exp(1/distance)
  3. Check collision
    1. Get distance from the behind vehicle of the lane
    2. Get velocity of the behind vehicle of the lane
    3. If the distance is small and the velocity is high, add the cost

2. Choose the lane according to the costs

3. If the distance to the car ahead is close, reduce the speed

4. Generate a trajectory based on the target lane and the target vehicle speed

