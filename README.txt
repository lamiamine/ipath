DATE: 

README for iPath: A C++ Library of Intelligent Global Path Planners for Mobile Robots with ROS Integration. [website: http://www.iroboapp.org/]

Authors/Contact: 
The list of all contributors is available at website: http://www.iroboapp.org/

I- Description:

iPath simulator implements the following planners: 
	1- A*
	2- Relaxed A* (RA*)
	3- Dijkstra
	4- Relaxed Dijkstra (RD)
	5- Genetic Algorithm (GA)
	6- Ant Colony Optimization (ACO)
	7- Tabu Search 


II- Directories:
- ipath which contains:
	+ ACO: A class that implements the ACO algorithm
	+ AStar: A class that implements the A* algorithm 
	+ evaluatePlanners
	+ generic_path_planner: A class that implements main methods for finding an initial solution; all path planners will inherit from it 
	+ genetic_algorithm: A class that implements the genetic algorithm 
	+ map_folder: contains the maps to be used and tested with the planners
	+ map: A class that represents a grid environment as a two dimensional matrix 
	+ path: A class that represents the path as a vector structure for the sequence of cells forming the path 
	+ RAstar: A class that implements relaxed A* algorithm
	+ tabu_search: A class that implements the tabu search 


III- iPath compilation and execution:

-To compile and run Astar planner:
1- In ipath directory run the command
$ make

2- Open file "evaluateASTAR.cpp" from ipath/evaluatePlanners/AStar directory, specify following parameters:

int numberOfScenarios = 2; // the number of the scenarios (start and goal positions)
int numberOfruns = 10; // how many times you want to repeat each scenario
// Astar parameters
bool withBreakTies = true;

3- run this command from ipath/evaluatePlanners/AStar directory
$ make

4- Open file "mapFile" from ipath/evaluatePlanners/AStar/bin directory. Add the map(s) you want to test, separate the maps by enter. For example:
map-W30-H30/W30-H30-ObRatio0.3-ObSize5-Nbr01.pgm
map-W50-H50/W50-H50-ObRatio0.1-ObSize1-Nbr01.pgm

5- run this command from ipath/evaluatePlanners/AStar/bin directory
$ ./main

6- Get the results from "AStarStatistics.xlsx" from ipath/evaluatePlanners/AStar/bin directory

-To compile and run Relaxed Astar planner:
1- In ipath directory run the command
$ make

2- Open file "evaluateRASTAR.cpp" from ipath/evaluatePlanners/RAstar directory, specify following parameters:

int numberOfScenarios = 2; // the number of the scenarios (start and goal positions)
int numberOfruns = 10; // how many times you want to repeat each scenario
// RAstar parameters
int numberOfNeighbors = 8; // 4 or 8
bool withBreakTies = true;

3- run this command from ipath/evaluatePlanners/RAstar directory
$ make

4- Open file "mapFile" from ipath/evaluatePlanners/RAstar/bin directory. Add the map(s) you want to test, separate the maps by enter. For example:
map-W30-H30/W30-H30-ObRatio0.3-ObSize5-Nbr01.pgm
map-W50-H50/W50-H50-ObRatio0.1-ObSize1-Nbr01.pgm

5- run this command from ipath/evaluatePlanners/RAstar/bin directory
$ ./main

6- Get the results from "RAStatistics.xlsx" from ipath/evaluatePlanners/AStar/bin directory


-To compile and run Genetic Algorithm planner:
1- In ipath directory run the command
$ make

2- Open file "evaluateGA.cpp" from ipath/evaluatePlanners/genetic_algorithm directory, specify following parameters:

int numberOfScenarios = 2; // the number of the scenarios (start and goal positions)
int numberOfruns = 10; // how many times you want to repeat each scenario
// GA parameters
int numberOfIterations = 15;
uint populationSize = 15;
int crossoverType = 1; // 1: one point, 2: two point, 3: modified.
float crossoverProbability = 0.9;
float mutationProbability = 0.01;
int mutationIterationNumber = 50;
float minInitialPathCost = 0;
int radius = 2;

3- run this command from ipath/evaluatePlanners/genetic_algorithm directory
$ make

4- Open file "mapFile" from ipath/evaluatePlanners/genetic_algorithm/bin directory. Add the map(s) you want to test, separate the maps by enter. For example:
map-W30-H30/W30-H30-ObRatio0.3-ObSize5-Nbr01.pgm
map-W50-H50/W50-H50-ObRatio0.1-ObSize1-Nbr01.pgm

5- run this command from ipath/evaluatePlanners/genetic_algorithm/bin directory
$ ./main

6- Get the results from "GAStatistics.xlsx" from ipath/evaluatePlanners/genetic_algorithm/bin directory

-To compile and run Tabu Search planner:
1- In ipath directory run the command
$ make

2- Open file "evaluateTABU.cpp" from ipath/evaluatePlanners/tabu_search directory, specify following parameters:

int numberOfScenarios = 2; // the number of the scenarios (start and goal positions)
int numberOfruns = 10; // how many times you want to repeat each scenario
// Tabu search parameters
int tenure=7;
int numberOfIterations = 30;

3- run this command from ipath/evaluatePlanners/tabu_search directory
$ make

4- Open file "mapFile" from ipath/evaluatePlanners/tabu_search/bin directory. Add the map(s) you want to test, separate the maps by enter. For example:
map-W30-H30/W30-H30-ObRatio0.3-ObSize5-Nbr01.pgm
map-W50-H50/W50-H50-ObRatio0.1-ObSize1-Nbr01.pgm

5- run this command from ipath/evaluatePlanners/tabu_search/bin directory
$ ./main

6- Get the results from "TabuSearchStatistics.xlsx" from ipath/evaluatePlanners/tabu_search/bin directory

-To compile and run all the planners:

1- Open file "main.cpp" from ipath directory, specify following parameters:

int numberOfScenarios = 2; // the number of the scenarios (start and goal positions)
int numberOfruns = 10; // how many times you want to repeat each scenario

And specify each planner parameters.

2- In ipath directory run the command
$ make

3- Open file "mapFile" from ipath/bin directory. Add the map(s) you want to test, separate the maps by enter. For example:
map-W30-H30/W30-H30-ObRatio0.3-ObSize5-Nbr01.pgm
map-W50-H50/W50-H50-ObRatio0.1-ObSize1-Nbr01.pgm

4- run this command from ipath/bin directory
$ ./main

5- Get the results from the "xlsx" files from ipath/bin directory
