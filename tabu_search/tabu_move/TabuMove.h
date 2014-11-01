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


#ifndef TABUMOVE_H
#define TABUMOVE_H

using namespace std;
/**
* @class TabuMove 
* @brief A class that presents the move applied to the path
*/
class TabuMove {
 
public:
  
	//default constructor to initialize path
	/**
	* @brief Constructor
	* @param cellS
	* @param cellG
	* @param tenure
	* @param iteration
	*/
	TabuMove(int cellS, int cellG, int tenure, int iteration);
	/**
	* @brief Constructor
	*/
	TabuMove(void);
	/**
	* @brief Destructor
	*/
	~TabuMove(void);
	
	//Define Mutators
	void setFromCell(int cell);
	void setToCell(int cell);
	void setTenure1(int t);
	void setDateOfExpiration(int iteration);
	
	//Define Accessors
	int getFromCell();
	int getToCell();
	int getTenure1();
	int getDateOfExpiration();
  
private:
  int fromCell;
  int toCell;
  int tenure;
  int dateOfExpiration;
};

#endif
