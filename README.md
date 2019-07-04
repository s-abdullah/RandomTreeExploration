# Rapidly Exploring Random Trees

This project implements the algorithm stated in this [paper](http://www.cs.columbia.edu/~allen/F18/NOTES/LavKuf01rrt.pdf) by Steven Lavalle.
The probkem is simple, given a state space and a goal, find the paht to the goal.


## Problem and Pseudo Code
This is the problem formulation as stated in the paper and the correspononding pseudo-code that is implemented.

![alt text][pr]

![alt text][ps]

## Inputs

### Obstacle File
* First integer gives you the number of obstacles
* For each obstacle:
	
	- First integer gives the number of vertices
	
	- The vertices follow as X Y pairs, one per line, each with two coordinates

### Position File
The text file containing the start and goal point has the format where the first line (X Y coordinates) determines the start point and second line (X Y coordinates) determines the goal point.

## The Program
###Usage
  Running bidrectional:

	python rrt.py world_obstacles.txt start_goal.txt -b

  Running single-direction:

	python rrt.py world_obstacles.txt start_goal.txt

### Methods
	**collision_free(st,end)**
	- checks that the passed edge does not collide with obstacles or
	tree edges
	- checks that the end point is within grid bounds as it is connected
	to a point in the tree that already meets reqs of free space

	**within_grid_bounds(pt)**
	- ensures a point is within grid bounds

	**draw(st, end, color=None)**
	- provides a simple call to the plot function
	- specifies a color in order to plot more than one tree/path

	**get_rand(range=None)**
	- returns a random point in the grid
	- range specifies a particular bias used once a node is near the goal

	**distance(p1, p2)**
	- returns euclidean distance between points

	**nearest_neighbor(q, T)**
	- returns tree node closest to specified point

	**new_state(q1, q2)**
	- returns a point from q1 in the direction of q2
	for a given step_size
	- checks against undefined and zero slope b/w two points

	**dijsktra(T, start, goal)**
	- returns the shortest path in a tree from start to goal

	**bidirectional_rrt(ts,tg)**
	- runs build_rrt twice simultaneously to find the shortest
	path to the goal faster

	The following two functions follow the guidelines of the rrt algorithm in the paper.
	Both implicitly use collision detection to plot points

	**build_rrt(T, q, goal, n=5000)**
	- Adds additional biases for the random points
	- Creates a random point and checks if it is within a step size
	of a tree node. If so, connects, else, calls extend
	- Calls extend to advance the tree a single node
	- If trapped, escapes by setting a random point elsewhere
	- Plots new edges that are within free space
	- returns all tree paths

	**extend(T, q)**
	- finds nearest neighbor and returns a new state in direction of
	q (passed random point)
	- only returns a value if advanced
	- reach state handled in build_rrt


[pr]: https://github.com/s-abdullah/RandomTreeExploration/blob/master/images/porb.png 
[ps]: https://github.com/s-abdullah/RandomTreeExploration/blob/master/images/ps.png 