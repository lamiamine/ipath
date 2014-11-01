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

#ifndef EVALUATEPLANNERS_H
#define EVALUATEPLANNERS_H

#include <stdio.h>
#include <cstdio>
#include <cstdlib>
#include<iostream>
#include <fstream>
#include <stdlib.h>
#include <time.h>    
#include <iomanip>
#include <sys/time.h>
#include "../AStar/AStar.h"
#include "../RAstar/RAStar.h"
#include "../genetic_algorithm/GA.h"
#include "../tabu_search/TabuSearch.h"

using namespace std;    

template <typename T>
/**
 * @class evaluatePlanners
 * @brief A class that used to evaluate different path planners
 */
class evaluatePlanners {
  
    
public:
	/**
	* @brief Constructor.
	*/  
	evaluatePlanners();
	/**
	* @brief Constructor.
	* @param statFile
	*/
	evaluatePlanners(string statFile);
	/**
	* @brief Destructor.
	*/
	~evaluatePlanners();

	void setStatFile(string statFile);
	string getStatFile();

	timespec diff(timespec start, timespec end);


	/* this function is responsible to run simulation for one scenario for a given planner 
	 * it generates one line in the stats file in the same format made by Adel Ammar to reuse his matlab code for analysis */
	/**
	* @brief this function is responsible to run simulation for one scenario for a given planner it generates one line in the stats file.
	* @param pathPlanner The path planner that should be evaluated
	* @param OGM The occupancy grid map that should be used
	* @param startCell The start cell
	* @param goalCell The goal cell
	*/
	void run(T* pathPlanner, OccupancyGridMap* OGM, int startCell, int goalCell);

	/* this function is responsible to run simulation for a set scenarios for a given planner 
	this function overload the run function above
	it also call it to evalute all scenarios all maps with all start and goal*/
	/**
	* @brief this function is responsible to run simulation for a set scenarios for a given planner, this function overload the run function above, it also call it to evalute all scenarios all maps with all start and goal.
	* @param pathPlanner The path planner that should be evaluated	
	* @param OGM The occupancy grid map that should be used
	* @param startGoalConfigArray The array that contains all start and goal cells
	* @param numberOfScenarios the number of the scenarios (start and goal positions)
	* @param numberOfruns how many times you want to repeat each scenario
	*/
	void run(T* pathPlanner,OccupancyGridMap* OGM, int** startGoalConfigArray, int numberOfScenarios, int numberOfruns);

	/** load maps into the vector 
	it reads the map_file and push all maps in that file into the mapFileArray**/

	/** load start and goal location into the matrix
	it reads the start_and_goal_file and push all start and goal in that file into the startGoalConfigArray[][]
	it is important to make sure that mapID c
	**/




	/** this should be initialized in evaluate_planner constructor
	* all planners are saved in map_file are added the array of maps*/

	string stats_file; //!< file where to output statistics. initialized in constructor.

	/** map_file contains the path and name of all maps to be evaluated.
	* each row in the file has a mapID, path to the map
	* initialized in constructor */

	string map_file;

	/*
	* start_and_goal_file contains several rows
	* each row has three values, mapID, start location, goal location
	* random Generation can be used to fill in this file
	* */
	//string start_and_goal_file;
	/* int startGoalConfigArray [][3] define the structure for start and goal for each map
	* each row contain the mapID, the start and goal
	* mapID is the index of the map in the mapFileArray */
 };
 
#endif 
