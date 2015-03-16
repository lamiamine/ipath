# iPath C++ Path Planning Library: Getting Started #
### Overview ###
This page provides an overview on how to get started with the iPath C++ path planner.<br>
A clone page is available on the iroboapp website at this link <a href='http://www.iroboapp.org/index.php?title=IPath'>http://www.iroboapp.org/index.php?title=IPath</a><br>

<h3>Authors</h3>
<ul><li><a href='http://dei.isep.ipp.pt/~akoubaa/'>Anis Koubaa</a>, COINS Research Group (Saudi Arabia)/Prince Sultan University (Saudi Arabia), akoubaa (at) coins-lab dot org <br>
</li><li>Imen Chaari, COINS Research Group (Saudi Arabia)/University of Mannouba (Tunisia), imen dot chaari (at) coins-lab dot org <br>
</li><li>Maram Alajlan, COINS Research Group (Saudi Arabia)/Al-Imam Mohamed bin Saud University (Saudi Arabia), maram dot ajlan (at) coins-lab dot org<br></li></ul>

<h3>Get the iPath Simulator Code</h3>
The code is open source under the GNU license. It can be downloaded from <br><a href='https://code.google.com/p/ipath/'>iPath</a>

<h3>API Documentation</h3>
<a href='http://www.iroboapp.org/ipath/api/docs/annotated.html'>API Documentation</a>

<h3>Description</h3>

iPath simulator implements the following planners:<br>
<ul><li>Astar<br>
</li><li>Relaxed Astar<br>
</li><li>Genetic Algorithm (GA)<br>
</li><li>Ant Colony Optimization (ACO)<br>
</li><li>Tabu Search <br></li></ul>


<h3>Directories</h3>
ipath which contains: <br>
<ul><li>ACO: A class that implements the ACO algorithm<br>
</li><li>AStar: A class that implements the Astar algorithm<br>
</li><li>evaluatePlanners<br>
</li><li>generic_path_planner: A class that implements main methods for finding an initial solution; all path planners will inherit from it<br>
</li><li>genetic_algorithm: A class that implements the genetic algorithm<br>
</li><li>map_folder: contains the maps to be used and tested with the planners<br>
</li><li>map: A class that represents a grid environment as a two dimensional matrix<br>
</li><li>path: A class that represents the path as a vector structure for the sequence of cells forming the path<br>
</li><li>RAstar: A class that implements relaxed A<b>algorithm<br>
</li><li>tabu_search: A class that implements the tabu search</li></ul></b>

<h3>iPath compilation and execution</h3>

<h4>To compile and run Astar planner</h4>
<ol><li>In ipath directory run the command<br>
</li></ol><blockquote><pre><code>$ make</code></pre></blockquote>

<ol><li>Open file <pre><code>evaluateASTAR.cpp</code></pre> from <pre><code>ipath/evaluatePlanners/AStar</code></pre> directory, specify following parameters:<br>
<pre><code><br>
int numberOfScenarios = 2; // the number of the scenarios (start and goal positions)<br>
int numberOfruns = 10; // how many times you want to repeat each scenario<br>
// Astar parameters<br>
bool withBreakTies = true;<br>
</code></pre>
</li><li>run this command from ipath/evaluatePlanners/AStar directory<br>
</li></ol><blockquote><pre><code>$ make</code></pre></blockquote>

<ol><li>Open file "mapFile" ipath/evaluatePlanners/AStar/bin directory<br>
Add the map(s) you want to test, separate the maps by enter. For example:<br>
</li></ol><blockquote>map-W30-H30/W30-H30-ObRatio0.3-ObSize5-Nbr01.pgm<br>
map-W50-H50/W50-H50-ObRatio0.1-ObSize1-Nbr01.pgm</blockquote>

<ol><li>run this command from <pre><code>ipath/evaluatePlanners/AStar/bin</code></pre> directory<br>
<pre><code> $ ./main </code></pre></li></ol>

<ol><li>Get the results from <pre><code>AStarStatistics.xlsx</code></pre> from <pre><code>ipath/evaluatePlanners/AStar/bin</code></pre> directory</li></ol>

<h4>To compile and run Relaxed Astar planner</h4>
1- In ipath directory run the command<br>
<pre><code>$ make</code></pre>

2- Open file <pre><code>evaluateRASTAR.cpp</code></pre> from <pre><code>ipath/evaluatePlanners/RAstar</code></pre> directory, specify following parameters:<br>
<br>
<blockquote>int numberOfScenarios = 2; // the number of the scenarios (start and goal positions)<br>
int numberOfruns = 10; // how many times you want to repeat each scenario<br>
// RAstar parameters<br>
int numberOfNeighbors = 8; // 4 or 8<br>
bool withBreakTies = true;</blockquote>

3- run this command from <pre><code>ipath/evaluatePlanners/RAstar</code></pre> directory<br>
<pre><code>$ make</code></pre>

4- Open file <pre><code>mapFile</code></pre> <pre><code>ipath/evaluatePlanners/RAstar/bin</code></pre> directory<br>
Add the map(s) you want to test, separate the maps by enter. For example:<br>
<pre><code><br>
map-W30-H30/W30-H30-ObRatio0.3-ObSize5-Nbr01.pgm<br>
map-W50-H50/W50-H50-ObRatio0.1-ObSize1-Nbr01.pgm<br>
</code></pre>
5- run this command from <pre><code>ipath/evaluatePlanners/RAstar/bin</code></pre> directory<br>
<pre><code> $ ./main </code></pre>

6- Get the results from <pre><code>RAStatistics.xlsx</code></pre> from <pre><code>ipath/evaluatePlanners/AStar/bin</code></pre> directory<br>
<br>
<br>
<h4>To compile and run Genetic Algorithm planner</h4>
1- In ipath directory run the command<br>
<pre><code> $ make </code></pre>

2- Open file <pre><code>evaluateGA.cpp</code></pre> from <pre><code>ipath/evaluatePlanners/genetic_algorithm</code></pre> directory, specify following parameters:<br>
<pre><code><br>
int numberOfScenarios = 2; // the number of the scenarios (start and goal positions)<br>
int numberOfruns = 10; // how many times you want to repeat each scenario<br>
// GA parameters<br>
int numberOfIterations = 15;<br>
uint populationSize = 15;<br>
int crossoverType = 1; // 1: one point, 2: two point, 3: modified.<br>
float crossoverProbability = 0.9;<br>
float mutationProbability = 0.01;<br>
int mutationIterationNumber = 50;<br>
float minInitialPathCost = 0;<br>
int radius = 2;<br>
</code></pre>
3- run this command from <pre><code>ipath/evaluatePlanners/genetic_algorithm</code></pre> directory<br>
<blockquote><pre><code>$ make</code></pre></blockquote>

4- Open file <pre><code>mapFile</code></pre> <pre><code>ipath/evaluatePlanners/genetic_algorithm/bin</code></pre> directory<br>
Add the map(s) you want to test, separate the maps by enter. For example:<br>
<pre><code><br>
map-W30-H30/W30-H30-ObRatio0.3-ObSize5-Nbr01.pgm<br>
map-W50-H50/W50-H50-ObRatio0.1-ObSize1-Nbr01.pgm<br>
</code></pre>
5- run this command from <pre><code>ipath/evaluatePlanners/genetic_algorithm/bin</code></pre> directory<br>
<pre><code> $ ./main </code></pre>

6- Get the results from <pre><code>GAStatistics.xlsx</code></pre> from <pre><code>ipath/evaluatePlanners/genetic_algorithm/bin</code></pre> directory<br>
<br>
<h4>To compile and run Tabu Search planner</h4>
1- In ipath directory run the command<br>
<pre><code>$ make</code></pre>

2- Open file <pre><code>evaluateTABU.cpp</code></pre> from <pre><code>ipath/evaluatePlanners/tabu_search</code></pre> directory, specify following parameters:<br>
<pre><code><br>
int numberOfScenarios = 2; // the number of the scenarios (start and goal positions)<br>
int numberOfruns = 10; // how many times you want to repeat each scenario<br>
// Tabu search parameters<br>
int tenure=7;<br>
int numberOfIterations = 30;<br>
</code></pre>
3- run this command from ipath/evaluatePlanners/tabu_search directory<br>
<pre><code>$ make</code></pre>

4- Open file "mapFile" ipath/evaluatePlanners/tabu_search/bin directory<br>
Add the map(s) you want to test, separate the maps by enter. For example:<br>
<pre><code><br>
map-W30-H30/W30-H30-ObRatio0.3-ObSize5-Nbr01.pgm<br>
map-W50-H50/W50-H50-ObRatio0.1-ObSize1-Nbr01.pgm<br>
</code></pre>
5- run this command from <pre><code>ipath/evaluatePlanners/tabu_search/bin</code></pre> directory<br>
<pre><code> $ ./main </code></pre>

6- Get the results from <pre><code>TabuSearchStatistics.xlsx</code></pre> from <pre><code>ipath/evaluatePlanners/tabu_search/bin</code></pre> directory<br>
<br>
<h4>To compile and run all the planners</h4>

1- Open file <pre><code>main.cpp</code></pre> from ipath directory, specify following parameters:<br>
<pre><code><br>
int numberOfScenarios = 2; // the number of the scenarios (start and goal positions)<br>
int numberOfruns = 10; // how many times you want to repeat each scenario<br>
</code></pre>
And specify each planner parameters.<br>
<br>
2- In ipath directory run the command<br>
<pre><code>$ make</code></pre>

3- Open file <pre><code>mapFile</code></pre> <pre><code>ipath/bin </code></pre> directory<br>
Add the map(s) you want to test, separate the maps by enter. For example:<br>
<pre><code><br>
map-W30-H30/W30-H30-ObRatio0.3-ObSize5-Nbr01.pgm<br>
map-W50-H50/W50-H50-ObRatio0.1-ObSize1-Nbr01.pgm<br>
</code></pre>
4- run this command from <pre><code>ipath/bin</code></pre> directory<br>
<pre><code> $ ./main</code></pre>

5- Get the results from the <pre><code>xlsx</code></pre> files from <pre><code>ipath/bin</code></pre> directory<br>
<br>
<h3>Screenshots</h3>

<h4>RAstar Execution</h4>

<img src='http://www.iroboapp.org/images/9/98/Rastar4.png' />

<img src='http://www.iroboapp.org/images/a/ad/Rastar5.png' />

<img src='http://www.iroboapp.org/images/e/e2/Rastar6.png' />

<img src='http://www.iroboapp.org/images/4/45/Rastar7.png' />

<h4>All planners Execution</h4>

<img src='http://www.iroboapp.org/images/4/48/Ipath2.png' />

<img src='http://www.iroboapp.org/images/c/c5/Ipath3.png' />

<img src='http://www.iroboapp.org/images/2/2f/Ipath4.png' />