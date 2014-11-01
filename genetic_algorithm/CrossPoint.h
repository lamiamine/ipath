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

#ifndef CROSSPOINT_H
#define CROSSPOINT_H


#include "../path/Path.h"
#include <algorithm> 
#include<vector>
using namespace std;
/**
* @class CrossPoint
* @brief A class that represents a common cell between two paths. To be used with crossover operator in the genetic algorithm
*/
class CrossPoint{

public:
	//constructor and destructor
	/**
	* @brief Constructor
	*/
	CrossPoint (void);
	/**
	* @brief Constructor
	* @param ID
	* @param CellIndex1 The index of the common cell in the first path
	* @param CellIndex2 The index of the common cell in the second path
	*/
	CrossPoint (int ID, int CellIndex1, int CellIndex2);
	/**
	* @brief Destructor
	*/
	~CrossPoint (void);

	 //Define Mutators
	void setCellID(int ID);
	void setPath1CellIndex (int CellIndex);
	void setPath2CellIndex (int CellIndex);

	//Define Accessors
	int getCellID();
	int getPath1CellIndex();
	int getPath2CellIndex();

	/**
	* @brief it used to find all the cross (common) points between two paths.
	* @param path1
	* @param path2
	* @return vector of all cross points between the two paths
	*/
	static vector <CrossPoint> findCrossPoints(Path* path1, Path* path2 );
	/**
	* @brief it used to choose randomly two of the cross points to perform the crossover with them
	* @param size Represent the number of the cross points
	* @return Two cross points
	*/
	static vector <int> chooseRandomlyTwoCrossPoints(uint size);
	/**
	* @brief it used to swap between two cross points
	*/
	void swapCrossPoints();

private:
	int cellID;
	int path1CellIndex;
	int path2CellIndex;
};

#endif

