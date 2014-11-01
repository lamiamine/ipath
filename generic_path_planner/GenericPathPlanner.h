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

#ifndef GENERICPATHPLANNER_H
#define GENERICPATHPLANNER_H


#include <stdio.h>
#include <algorithm> 
#include "../path/Path.h"

/**
 * @class GenericPathPlanner
 * @brief A class that implements main methods for finding an initial solution; all path planners will inherit from it.
 */
class GenericPathPlanner{
  
public:
	/*constructor and destructor*/ 
	/**
	* @brief Constructor.
	*/
	GenericPathPlanner(void);
	/**
	* @brief Constructor.
	* @param numIterations
	*/
	GenericPathPlanner(int numIterations);
	/**
	* @brief Destructor.
	*/
	virtual ~GenericPathPlanner(void); 
	
	void setNumberOfIterations (int numIterations);
	int getNumberOfIterations ();
	
	/*utilities methods needed for search*/
	/**
	* @brief it is used to find the free neighbors Cells of a the current Cell in the grid
	* @param map
	* @param CellID
	* @return a vector of free neighbor cells of the current cell
	*/
	vector <int> findFreeNeighborCell (OccupancyGridMap* map,int CellID);
	/**
	* @brief it is used to find the free four neighbors Cells of a the current Cell in the grid
	* @param map
	* @param CellID 
	* @return a vector of free four neighbor cells of the current cell
	*/
	vector <int> findfourFreeNeighborCell (OccupancyGridMap* map,int CellID);
	/**
	* @brief it is used to find the neighbors Cells of a the current Cell in the grid
	* @param map
	* @param CellID  
	* @return a vector of free neighbor cells of the current cell
	*/
	vector <int> findNeighborCell (OccupancyGridMap* map,int CellID);
	/**
	* @brief it is to remove the already visited neighbor from the path
	* @param neighborCells
	* @param path
	* @return a vector of unvisited neighbor
	*/
	vector <int> getUnvisitedFreeNeighbors (vector <int> neighborCells, Path *path);
	/**
	* @brief it used to search the neighbor cells that are non deadlock 
	* @param CellID
	* @param unvisitedNeighborCells
	* @param deadLockCells
	* @return a vector of neighbor cells 
	*/
	vector <int> getNonDeadlockFreeNeighbors (int CellID, vector <int> unvisitedNeighborCells, vector <vector <int> > deadLockCells);
	/**
	* @brief it is used to find neighbor Cells that are not visited and that will not lead to a deadlock
	* @param map
	* @param path
	* @param currentCell
	* @param deadLockCells
	* @param goalCell
	* @return a vector of neighbor Cells 
	*/
	vector <int> getValidNeighbors(OccupancyGridMap* map,Path* path, int currentCell, vector <vector <int> > deadLockCells, int goalCell );
	/**
	* @brief it calculates the Move cost to the neighbors and return the best next cell
	* @param map
	* @param currentCell
	* @param neigborCells
	* @return the next cell
	*/
	int getBestNextCell(OccupancyGridMap* map,int currentCell, vector<int> neigborCells);
	/**
	* @brief it generates the next cell smartly if isSmart is true or by greedy method if isSmart is false
	* @param map
	* @param currentCell
	* @param neigborCells
	* @param goalCell
	* @param isSmart
	* @return the next cell
	*/
	int getBestNextCellSmartly(OccupancyGridMap* map,int currentCell, vector<int> neigborCells, int goalCell, bool isSmart);
	/**
	* @brief it generates the next cell by backtracking
	* @param map
	* @param deadLockPos
	* @param path
	* @param goalCell
	* @return the next cell
	*/
	int backtracking(OccupancyGridMap* map, vector <vector <int> > &deadLockPos, Path *& path, int goalCell);
	/**
	* @brief it generates the next cell using Euclidian distance
	* @param map
	* @param neighborCells
	* @param goalCell
	* @return the next cell
	*/
	int getBestNextCellEuclidian(OccupancyGridMap* map, vector<int> neighborCells, int goalCell);
	/**
	* @brief it generates the next cell using Manhattan distance
	* @param map
	* @param neighborCells
	* @param goalCell 
	* @return the next cell
	*/
	int getBestNextCellManhattan(OccupancyGridMap* map, vector<int> neighborCells, int goalCell);
	
	
	/*core methods*/
	/**
	* @brief it generates the initial path
	* @param map
	* @param startCell
	* @param goalCell
	* @param isSmart
	* @return the initial path
	*/
	Path* getInitialPath(OccupancyGridMap* map,int startCell, int goalCell, int isSmart);
	/**
	* @brief it generates the robot path
	* @param map
	* @param initialPath
	* @return the robot path
	*/
	virtual Path* findPath(OccupancyGridMap* map, Path* initialPath); 
	/**
	* @brief it generates the robot path
	* @param map
	* @param initialPath
	* @param time
	* @return the robot path
	*/
	virtual Path* findPath(OccupancyGridMap* map, Path* initialPath, timespec & time);
	/**
	* @brief it generates the robot path
	* @param map
	* @param startCell
	* @param goalCell
	* @return the robot path
	*/
	virtual Path* findPath(OccupancyGridMap* map,int startCell, int goalCell);
	/**
	* @brief it generates the robot path
	* @param map
	* @param startCell
	* @param goalCell
	* @param withBreakTies
	* @param neighborNumber
	* @return the robot path
	*/
	virtual Path* findPath(OccupancyGridMap* map,int startCell, int goalCell, bool withBreakTies,int neighborNumber);
	/**
	* @brief it is used to draw a staright line between the start and goal positions
	* @param OGM
	* @param startCell
	* @param goalCell
	* @return an initial path (feasible or unfeasible)
	*/
	Path* drawStraightLineStartGoal (OccupancyGridMap* OGM,int startCell, int goalCell);
	/**
	* @brief it is used to find neighbors of current cell at radius N
	* @param map
	* @param neighborhoodRadius
	* @param centerCell
	* @return a vector of neighbors at radius N
	*/
	vector <int> getNeighborsAtRadiusN(OccupancyGridMap* map,int neighborhoodRadius,int centerCell);
	
	/*predicate methods*/
	/**
	* @brief check if the start and goal cells are valid
	* @param map
	* @param startCell
	* @param goalCell
	* @return true if the start and the goal cells are valid, false otherwise
	*/
	bool isStartAndGoalCellsValid(OccupancyGridMap* map,int startCell,int goalCell);
	/**
	* @brief it verify if the couple(cellID, neighbor) exists in the DeadLock vector or not
	* @param CellID
	* @param unvisitedNeighborCell
	* @param deadLockCells
	* @return true if the couple(cellID, neighbor) exists in the DeadLock vector or not, false otherwise
	*/
	bool isDeadlock(int CellID, int unvisitedNeighborCell, vector <vector <int> > deadLockCells);
	/**
	* @brief it is to verify if a cell exists in the path or not
	* @param CellID
	* @param path
	* @return true if the cell exists in the path or not, false otherwise
	*/
	bool isVisited(int CellID, Path *path);
	/**
	* @brief check if the cell exits in the vector
	* @param CellID
	* @param vect
	* @return true if the cell exits in the vector, false otherwise
	*/
	bool contains(int CellID, vector<int> vect);
	/**
	* @brief To check if two paths are equal or not
	* @param map
	* @param path1
	* @param path2
	* @return true if the two paths equal, false otherwise
	*/
	bool areTwoPathsEqual(OccupancyGridMap* map, Path* path1, Path* path2);

	
private:
	int numberOfIterations; //!<
};

#endif
