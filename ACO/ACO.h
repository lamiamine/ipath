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
#include <algorithm> 
#include <math.h> 
#include <numeric>
#include <list>
#include "pheromone.h"
#include "GenericACO.h"
#include <vector>


using std::vector ;

/**
 * @class ACO
 * @brief A class that implements ---
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
	* @param pheromoneL
	*/
	ACO(list <pheromone> pheromoneL);
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
 	ACO(long double initialPheromoneValue,long double Alpha , long double Beta , long double EvaporationRate ,int Q , unsigned int NumberOfAnts,int NumberOfIterations);
	/**
	* @brief Destructor.
	*/
 	~ACO(void);

	 list <pheromone> getPheromoneList();
	 void setPheromoneList (list <pheromone> PeromoneList);
	/**
	* @brief it used to initialze the list of pheromone
	* @param map
	*/	 
	 void initializePheromoneList(OccupancyGridMap* map);
	/**
	* @brief it calculates the transition probabilities of the ants between the cells
	* @param map
	* @param CurrentCell
	* @param NeighborCell
	* @param iteration
	* @return the probabilities calculated by the transition rule probability
	*/	 
	 vector < long double > computeTransitionProbabilities(OccupancyGridMap* map, int CurrentCell, vector <int> NeighborCell, int iteration);
	/**
	* @brief it used to obtain a pheromone value of an arc from the pheromone list
	* @param FromCell
	* @param ToCell
	* @return the pheromone value on the edge (FromCell,ToCell)
	*/	
	 long double getPheromoneValue(int FromCell,int ToCell);
	/**
	* @brief it is used to update the quantity of pheromone.
	* @param ants
	*/		 
	 void updatePheromone(vector <Ant*> ants);
	/**
	* @brief It is used to calculate the delta value used in the update pheromone rule.
	* @param ants
	* @param cell1
	* @param cell2
	* @return the delta value
	*/	
	 long double getDelta(vector <Ant*> ants,int cell1,int cell2);
	/**
	* @brief it is used to select the next cell of the ant
	* @param NeighborCell
	* @param probabilities
	* @return the next cell of the ant
	*/	
	 int getNextCell(vector <int> NeighborCell, vector < long double > probabilities);
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
  list <pheromone> PheromoneList; //!< the pheromone list 

};

#endif
