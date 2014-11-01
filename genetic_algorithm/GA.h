/* iPath: A C++ Library of Intelligent Global Path Planners for Mobile Robots with ROS Integration. 

 * Website: http://www.iroboapp.org/index.php?title=IPath
 * Contact: 
 *
 * Copyright (c) 2014
 * Owners: Al-Imam University/King AbdulAziz Center for Science and Technology (KACST)/Prince Sultan University
 * All rights reserved.
 *
 * License Type: GNU GPL
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef GA_H
#define GA_H
#include "../generic_path_planner/GenericPathPlanner.h"
#include <sstream>
#include "Population.h"
#include "CrossPoint.h"

/**
* @class GA 
* @brief A class that implements the genetic algorithm  
*/
class GA : public GenericPathPlanner
{
public:
	/**
	* @brief Constructor
	*/
	GA (void);
	/**
	* @brief Constructor
	* @param numberOfIterations The number of iterations or generations
	* @param populationS The population size
	* @param crossType The crossover type
	* @param crossProb The crossover probability
	* @param mutationProb The mutation probability
	* @param mutationIterationNumber The number of mutation tries.
	* @param minInitialPathCost The minimum cost allowed to the paths. 
	* @param radius_ The farthest distance from the cells in the initial path to intermediate cell which will be used to generate a new path from start to goal position across the selected intermediate cell
	*/
	GA (int numberOfIterations, uint populationS, int crossType, float crossProb, float mutationProb, int mutationIterationNumber, float minInitialPathCost, int radius_);
	/**
	* @brief Constructor
	* @param numberOfIterations The number of iterations or generations
	* @param populationS The population size
	* @param crossProb The crossover probability
	* @param mutationProb The mutation probability
	* @param mutationIterationNumber The number of mutation tries.
	* @param radius_ The farthest distance from the cells in the initial path to intermediate cell which will be used to generate a new path from start to goal position across the selected intermediate cell
	*/
	GA (int numberOfIterations, uint populationS, float crossProb, float mutationProb, int mutationIterationNumber, int radius_);
	/**
	* @brief Destructor
	*/
	~GA (void);

	void setPopulationSize(uint populationS);
	void setCrossoverType(int crossType);
	void setCrossoverProbability(float crossProb);
	void setMutationProbability(float mutationProb);
	void setMutationIteration(int mutationIterationNumber);
	void setMinimumInitialPathCost (float minimumInitialPathCost);
	void setRadius(int radius);

	uint getPopulationSize();
	int getCrossoverType();
	float getCrossoverProbability();
	float getMutationProbability();
	int getMutationIteration();
	float getMinimumInitialPathCost();
	int getRadius();

	/**
	* @brief it is used to generate a set of initial paths from start cell to goal cell
	* @param OGM The occupancy grid map that should be used
	* @param startCell The start cell
	* @param goalCell The goal cell
	* @return initial population
	*/
	Population* getInitialPopulation(OccupancyGridMap* OGM, int startCell, int goalCell);

	/****************************************************/
	// selection methods
	/****************************************************/
	/**
	* @brief evaluate the path fitness
	* @param OGM The occupancy grid map that should be used
	* @param path The path that should be evaluated
	* @return the path fitness
	*/
	int evaluatePathFitness(OccupancyGridMap* OGM, Path* path);
	/**
	* @brief it is used to select the best paths based on the rank selection
	* @param OGM The occupancy grid map that should be used
	* @param population The population that should be used in the selection
	* @return return a selected set of paths (equal to the size of population)
	*/
	Population* rankSelection(OccupancyGridMap* OGM, Population* population);
	/**
	* @brief sort the path fitness in ascending order [insertion sort]
	* @param pathIndices a reference to path indices vector
	* @param fitness a reference to fitness vector
	*/
	void sort(vector <int> &pathIndices, vector <int> &fitness);

	/****************************************************/
	// crossover methods
	/****************************************************/
	/**
	* @brief call appropriate crossover function based on crossoverType.
	* @param OGM The occupancy grid map that should be used
	* @param path1 The first path participate in the crossover
	* @param path2 The first second participate in the crossover
	* @return the resulting paths from the crossover
	*/
	vector <Path*> crossover (OccupancyGridMap* OGM, Path* path1, Path* path2);
	/**
	* @brief it is used to prepare two paths for two-point crossover
	* @param OGM The occupancy grid map that should be used
	* @param path1 The first path participate in the crossover
	* @param path2 The first second participate in the crossover
	* @return the resulting paths from the crossover
	*/
	vector <Path*> twoPointCrossover(OccupancyGridMap* OGM, Path* path1, Path* path2);
	/**
	* @brief it is used to prepare two paths for one-point crossover
	* @param OGM The occupancy grid map that should be used
	* @param path1 The first path participate in the crossover
	* @param path2 The first second participate in the crossover
	* @return the resulting paths from the crossover
	*/
	vector <Path*> onePointCrossover(OccupancyGridMap* OGM, Path* path1, Path* path2);
	/**
	* @brief it is used to prepare two paths for smart crossover
	* @param OGM The occupancy grid map that should be used
	* @param path1 The first path participate in the crossover
	* @param path2 The first second participate in the crossover
	* @return the resulting paths from the crossover
	*/
	vector <Path*> smartCrossover(OccupancyGridMap* OGM,Path* path1, Path* path2);

	/**
	* @brief it is used to perform two-point crossover
	* @param OGM The occupancy grid map that should be used
	* @param path1 The first path participate in the crossover
	* @param path2 The first second participate in the crossover
	* @param point1 The first crossover point
	* @param point2 The second crossover point
	* @return the resulting path from the crossover
	*/
	Path* crossoverPath(OccupancyGridMap* OGM, Path* path1, Path* path2, CrossPoint point1, CrossPoint point2);
	/**
	* @brief it is used to perform one-point crossover
	* @param OGM The occupancy grid map that should be used
	* @param path1 The first path participate in the crossover
	* @param path2 The first second participate in the crossover
	* @param point Crossover point
	* @return the resulting path from the crossover
	*/
	Path* crossoverPath(OccupancyGridMap* OGM, Path* path1, Path* path2,  CrossPoint point);
	/**
	* @brief it is used to perform smart crossover
	* @param OGM
	* @param smartPath The path that will hold the best part bewteen cell1 and cells from path1
	* @param path1 The path that hold the best part between cell1 and cell1
	* @param cell1 The first crossover point
	* @param cell2 The second crossover point
	* @return the resulting path from the crossover
	*/
	Path* crossoverPath(OccupancyGridMap* OGM, Path* smartPath, Path* path1, int cell1, int cell2);
	/**
	* @brief To check if the crossover can be performed on the two paths; The crossover is feasible if: 1- the paths are feasible 2- the paths are different 3- the paths have same start and goal
	* @param OGM The occupancy grid map that should be used
	* @param path1 The first path participate in the crossover
	* @param path2 The first second participate in the crossover
	* @return true if the crossover feasible, false otherwise
	*/
	bool isCrossoverFeasible(OccupancyGridMap* OGM, Path* path1, Path* path2);
	/**
	* @brief Check if the two common cells in inverted position
	* @param Point1 The first common point
	* @param Point2 The second common point
	* @return true if they inverted, false otherwise
	*/
	bool areCommonCellsInvert(CrossPoint Point1, CrossPoint Point2);

	/****************************************************/
	// mutation methods
	/****************************************************/
	/**
	* @brief it used to perform the mutation operator
	* @param OGM The occupancy grid map that should be used
	* @param path The path that should be used for mutation
	* @return path after mutation
	*/
	Path* mutatePath(OccupancyGridMap* OGM, Path* path);
	/**
	* @brief it is used to perform the mutation on one cell
	* @param OGM The occupancy grid map that should be used
	* @param path The path that should be used for mutation
	* @return path after mutation
	*/
	Path* mutateCell(OccupancyGridMap* OGM, Path* path);
	/**
	* @brief  it is used to perform the mutation on sub path cell
	* @param OGM The occupancy grid map that should be used
	* @param path The path that should be used for mutation 
	* @return path after mutation
	*/
	Path* mutateSubPath(OccupancyGridMap* OGM, Path* path);
	/**
	* @brief  it is used to generate a path between cell1 and cell2
	* @param OGM The occupancy grid map that should be used
	* @param cell1 Start cell in the sub path 
	* @param cell2 Goal cell in the sub path
	* @return path between cell1 and cell2
	*/
	Path* findSubPath(OccupancyGridMap* OGM, int cell1, int cell2);
	/**
	* @brief it is used to choose one cell from a path randomly
	* @param path The path that should be used
	* @return one cell randomly
	*/
	uint getRandomCell(Path* path);

	/****************************************************/
	/**
	* @brief it is used to perform the genetic algorithms operator.
	* @param OGM The occupancy grid map that should be used
	* @param pop The population that should be used
	* @param bestExecutionTime The time needed to find the best path
	* @return best path found by GA
	*/
	Path* findPath(OccupancyGridMap* OGM, Population* pop,timespec & bestExecutionTime);
	/**
	* @brief it is used to find a best path in a population
	* @param OGM The occupancy grid map that should be used
	* @param pop The population that should be used
	* @return index of the shortest path in pop
	*/
	int findBestPath(OccupancyGridMap* OGM, Population* pop);
	/**
	* @brief it is used to calculate the execution time
	* @param start starting time
	* @param end ending time
	* @return execution time
	*/
	timespec diff(timespec start, timespec end);

private:
	uint populationSize; //!< The number of paths in one population
	int crossoverType; //!< To specify the crossover type; 1: one-point crossover; 2: two-point crossover; 3: modified crossover; the default is 1.
	float crossoverProbability; //!< [0 - 1], typically it should be in range [0.7 - 1]
	float mutationProbability; //!< [0 - 1], typically it should be in range [0 - 0.01]
	int mutationIteration; //!< The number of mutation tries; 
	float minimumInitialPathCost; //!< The minimum cost allowed to the paths; Default is 0
	int radius; //!< The farthest distance from the cells in the initial path to intermediate cell which will be used to generate a new path from start to goal position across the selected intermediate cell

};

#endif
