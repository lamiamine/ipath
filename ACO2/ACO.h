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


#ifndef ACO_H
#define ACO_H

#include <stdio.h>
#include "GenericACO.h"
#include <math.h> 
#include <numeric>

/**
 * @class ACO
 * @brief neew class that implements -------------
 */
class ACO: public GenericACO
{
public:
	/**
	* @brief Constructor.
	*/
	ACO(void);
	/**
	* @brief Constructor.
	* @param initialPheromoneValue
	* @param Alpha
	* @param Beta
	* @param EvaporationRate
	* @param Q
	* @param NumberOfAnts
	* @param NumberOfIterations
	*/
	ACO( long double initialPheromoneValue, long double Alpha ,  long double Beta ,  long double EvaporationRate ,int Q , unsigned int NumberOfAnts,int NumberOfIterations);
	/**
	* @brief Constructor.
	* @param PM
	*/
	ACO( long double** PM);
	/**
	* @brief Destructor.
	*/
	~ACO(void);
	
	void setPheromoneMatrix( long double** PM);
	long double** getPheromoneMatrix();
	/**
	* @brief it used to initialze the Matrix of pheromone 
	* @param map
	*/
	void initializePheromoneMatrix(OccupancyGridMap* map);
	/**
	* @brief it calculates the transition probabilities of the ants between the cells
	* @param map
	* @param CurrentCell
	* @param NeighborCell
	* @return the probabilities calculated by the transition rule probability
	*/
	vector <long double > computeTransitionProbabilities(OccupancyGridMap* map, int CurrentCell, vector <int> NeighborCell);
	/**
	* @brief it is used to select the next cell of the ant
	* @param NeighborCells
	* @param probabilities
	* @return the next cell of the ant
	*/
	int getNextCell(vector <int> NeighborCells, vector <long double> probabilities);
	/**
	* @brief it is used to update the quantity of pheromone.
	* @param map
	* @param ants
	*/
	void updatePheromone(OccupancyGridMap* map, vector <Ant*> ants);
	/**
	* @brief It is used to calculate the delta value used in the update pheromone rule.
	* @param ants
	* @param cell
	* @return the delta value
	*/
	long double getDelta(vector <Ant*> ants,int cell);
	/**
	* @brief it is used to generate the best path after N iterations 
	* @param map
	* @param startCell
	* @param goalCell
	* @param enable_trace
	* @return the best path of the algorithm
	*/
	Path* findPath(OccupancyGridMap* map, int startCell, int goalCell, bool enable_trace); 

 
private:
    long double** pheromoneMatrix; //!<  the pheromone Matrix  
};

#endif
