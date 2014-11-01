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


#ifndef RASTAR_H
#define RASTAR_H

#include<vector>
#include <list>
#include "../generic_path_planner/GenericPathPlanner.h"

using namespace std;
/**
 * @struct cells
 * @brief A struct that represents a cell and its fCost.
 */
struct cells {
	int currentCell;
	float fCost;
};      

/**
 * @class RAStar
 * @brief A class that implements relaxed A* algorithm.
 */
class RAStar: public GenericPathPlanner{

public:
	
	/**************Constructors and destructor********/
	/**
	* @brief Constructor.
	*/
	RAStar(void);
	/**
	* @brief Constructor.
	* @param NumberOfNeighbors
	* @param BT
	*/
	RAStar(int NumberOfNeighbors,bool BT);
	/**
	* @brief Destructor.
	*/    
	~RAStar(void); 
	 
	
	/**************Mutators**********/
	void setOpenList(list<cells> OPL);
	void setGScore(float** gScore);
	void setNeighborNumber(int neighbor);
	void setIsWithBreakTies(bool withBT);
	  
	list<cells> getOpenList();
	float ** getGScore();
	int getNeighborNumber();
	bool getIsWithBreakTies();
	
	/*******core methods*************/
	
	/**
	* @brief  it is used to add the neighbor Cells to the open list
	* @param map The occupancy grid map  that should be used
	* @param OPL A reference to the Open List
	* @param neighborCell Set of the neighbor cells
	* @param g_score 
	* @param goalCell the goal cell
	* @param tBreak
	*/	
	void addNeighborCellToOpenList(OccupancyGridMap* map, list<cells> & OPL, int neighborCell, float** g_score,int goalCell,float tBreak);
	/**
	* @brief it is used to check if a cell exists in the open list or in the closed list
	* @param list
	* @param cellID
	* @return true if the cell ID is in the list, false otherwise
	*/
	bool isContains(list<cells> & list, int cellID);
	/**
	* @brief it is used to calculate the hCost 
	* @param map
	* @param cellID
	* @param goalCell
	* @return the distance between the current cell and the goal cell
	*/
	float calculateHCost(OccupancyGridMap* map,int cellID, int goalCell);
	/**
	* @brief it is used to search the index of a cell in a list
	* @param list1
	* @param cellID
	* @return index of the cell in the list
	*/
	list<cells>::iterator getPositionInList(list<cells> & list1, int cellID);
	/**
	* @brief comparaison function
	* @param c1
	* @param c2
	* @return True is fCost of c1 less than fCost of c2, false otherwise
	*/
	static bool _compareFCost(cells const &c1, cells const &c2);
	/**
	* @brief it is used to construct the robot path
	* @param map
	* @param g_score
	* @param startCell
	* @param goalCell
	* @return the best path
	*/
	Path* constructPath(OccupancyGridMap* map,float** g_score, int startCell, int goalCell);
	/**
	* @brief it is used to generate the robot free path
	* @param map
	* @param startCell
	* @param goalCell
	* @return the best path
	*/
	Path* findPath(OccupancyGridMap* map, int startCell, int goalCell);
  
private:
   
    list<cells> openList; //!< the open list: it contains all the visited cells (current cells)
    float ** g_score; //!<
    int neighborNumber; //!<
    bool isWithBreakTies; //!<
    
};

#endif
