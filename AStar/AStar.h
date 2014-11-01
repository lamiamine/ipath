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

#ifndef ASTAR_H
#define ASTAR_H

#include<vector>
#include <list>
#include "../generic_path_planner/GenericPathPlanner.h"

using namespace std;
/**
 * @struct coupleOfCells
 * @brief A struct that represents a couple of current and parent cells 
 */
struct coupleOfCells {
	int currentCell;
	int parentCell; 
	float gCost;
	float hCost;
	float fCost;
};      
/**
 * @class AStar
 * @brief A class that implements the A* algorithm.
 */
class AStar: public GenericPathPlanner{
	public:
    
  
	/**************Constructors and destructor********/
	/**
	* @brief Constructor.
	* @param BT
	*/
	AStar(bool BT);
	/**
	* @brief Destructor.
	*/
	~AStar(void); 
	
	
	/**************Mutators**********/
	void setOpenList(list<coupleOfCells> OPL);
	void setClosedList(list<coupleOfCells> CSL);
	void setIsWithBreakTies(bool withBT);
	
	list<coupleOfCells> getOpenList();
	list<coupleOfCells> getClosedList();
	bool getIsWithBreakTies();
	
	/*******core methods*************/
	/**
	* @brief  it is used to add the neighbor Cells to the open list
	* @param map The occupancy grid map  that should be used
	* @param OPL A reference to the Open List
	* @param neighborCells Set of the neighbor cells
	* @param parent the parent of the neighbor cells
	* @param gCostParent gCost of the parent
	* @param goalCell the goal cell
	* @param tBreak
	*/
	void addNeighborCellsToOpenList(OccupancyGridMap* map, list<coupleOfCells> & OPL, vector <int> neighborCells, int parent, float gCostParent, int goalCell,float tBreak);
	/**
	* @brief it is used to check if a cell exists in the open list or in the closed list
	* @param list
	* @param cellID
	* @return true if the cell ID is in the list, false otherwise
	*/
	bool isContains(list<coupleOfCells> & list, int cellID);
	/**
	* @brief it is used to calculate the hCost 
	* @param map
	* @param cellID
	* @param goalCell
	* @return the distance between the current cell and the goal cell
	*/
	float calculateHCost(OccupancyGridMap* map,int cellID, int goalCell);
	/**
	* @brief it is used to calculate the new gCost of the neighbor cells that exist in the open list and compare them to the old gCost
	* @param map
	* @param OPL
	* @param neighborCellsInOpenList
	* @param currentCell
	* @param gCostCurrent
	*/
	void recalculateGCost(OccupancyGridMap* map, list<coupleOfCells> & OPL, vector<int> neighborCellsInOpenList,int currentCell, float gCostCurrent);
	/**
	* @brief it is used to search the index of a cell in a list
	* @param list1
	* @param cellID
	* @return index of the cell in the list
	*/	
	list<coupleOfCells>::iterator getPositionInList(list<coupleOfCells> & list1, int cellID);
	/**
	* @brief comparaison function
	* @param c1
	* @param c2
	* @return True is fCost of c1 less than fCost of c2, false otherwise
	*/
	static bool _compareFCost(coupleOfCells const &c1, coupleOfCells const &c2);
	/**
	* @brief it is used to construct the robot path
	* @param map
	* @param CSL
	* @param startCell
	* @param goalCell
	* @return the best path
	*/
	Path* constructPath(OccupancyGridMap* map,list<coupleOfCells> & CSL, int startCell, int goalCell);
	/**
	* @brief it is used to generate the robot free path
	* @param map
	* @param startCell
	* @param goalCell
	* @return the best path
	*/
	Path* findPath(OccupancyGridMap* map, int startCell, int goalCell);
		    
private:
   
	list<coupleOfCells> openList; //!< the open list: it contains all the visited cells (current cells)
	list<coupleOfCells> closedList; //!< the closed list   
	bool isWithBreakTies; //!<
};

#endif
