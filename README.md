# Comparison of Search based Vs Sampling based motion planning algorithms

## Objective:
The objective of this project is to compare search based and sampling based motion planning algortihms.
In this project A* and RRT algorithms are compared. The A* algorithm's performance is further analysed by varying the heuristic fuctions. The comparison is done on 7 different environoments with 3-D obstracles.

## A* Vs RRT
- Below given images are of A* (left image) RRT(right image)
<div style="display: flex; justify-content: center;">
  <img src="images/maze_astar.png" width="400" alt="A *" style="margin-right: 20px;">
  <img src="images/maze_rrt.png" width="400" alt="RRT">
</div>
<br>
<div style="display: flex; justify-content: center;">
  <img src="images/window_astar.png" width="400" alt="A *" style="margin-right: 20px;">
  <img src="images/window_rrt.png" width="400" alt="RRT">
</div>
<br>
<div style="display: flex; justify-content: center;">
  <img src="images/tower_astar.png" width="400" alt="A *" style="margin-right: 20px;">
  <img src="images/tower_rrt.png" width="400" alt="RRT">
</div>
<br>
<div style="display: flex; justify-content: center;">
  <img src="images/room_astar.png" width="400" alt="A *" style="margin-right: 20px;">
  <img src="images/room_rrt.png" width="400" alt="RRT">
</div>
<br>
<div style="display: flex; justify-content: center;">
  <img src="images/monza_astar.png" width="400" alt="A *" style="margin-right: 20px;">
  <img src="images/monza_rrt.png" width="400" alt="RRT">
</div>
<br>
<div style="display: flex; justify-content: center;">
  <img src="images/flappy_bird_astar.png" width="400" alt="A *" style="margin-right: 20px;">
  <img src="images/flappy_bird_rrt.png" width="400" alt="RRT">
</div>
<br>


## Results:
<div style="display: flex; justify-content: center;">
  <img src="images/Astar.png" width="400" alt="A *" style="margin-right: 5px;">
  <img src="images/RRTVSAstar.png" width="400" alt="RRT">
</div>

## Code and Installation:
1. Install the requirements
```bash
conda create --name env_motionPlanning
conda activate env_motionPlanning
git clone https://github.com/suryapilla/motion-planning.git
cd motion-planning
pip install -r requirements.txt

```
2. Adjust the config file to speciy heuristic, planner, environment or use the below command
```
python3 main.py --env <env name> --planner <choose planner>

```

Example: To choose tower environmnet and Astart planner use below command
```
python3 main.py --env tower --planner Astar
```

