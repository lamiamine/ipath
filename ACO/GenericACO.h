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


#ifndef GENERICACO_H
#define GENERICACO_H

#include <stdio.h>
#include "Ant.h" 
#include <vector>

using namespace std;
/**
 * @class GenericACO
 * @brief A class that implements ---
 */
class GenericACO: public GenericPathPlanner
{
public:
	/**
	* @brief Constructor
	*/
	GenericACO(void);
	/**
	* @brief Constructor
	* @param initialPheromoneValue
	* @param Alpha
	* @param Beta
	* @param EvaporationRate
	* @param Q
	* @param NumberOfAnts
	* @param NumberOfIterations	
	*/
	GenericACO(long double initialPheromoneValue, long double Alpha , long double Beta , long double EvaporationRate ,int Q , unsigned int NumberOfAnts,int NumberOfIterations);
	/**
	* @brief Destructor.
	*/
	~GenericACO(void);
	
	long double getInitialPheromoneValue();
	void setInitialPheromoneValue (long double InitialePheromoneValue);
	long double getAlpha();
	void setAlpha(long double Alpha);
	long double getBeta();
	void setBeta(long double Beta);
	long double getEvaporationRate();
	void setEvaporationRate(long double EvaporationRate);
	int getQ();
	void setQ(int Q);
	unsigned int getNumberOfAnts();
	void setNumberOfAnts(unsigned int NumberOfAnts);
	/**
	* @brief it is to generate the best path from a set of candidate paths
	* @param map
	* @param paths
	* @return index of the best path in the vector
	*/
	int findBestPath(OccupancyGridMap* map, vector<Path*> paths);
	/**
	* @brief it is to generate the best ant from a set of candidate ants  
	* @param map
	* @param Ants
	* @return index of the best Ant in the vector
	*/
	Path* findBestAnt(OccupancyGridMap* map, vector<Ant*> Ants);

  
private:
 
  long double initialPheromoneValue ; //!< the initial pheromone value
  unsigned int numberOfIterations ; //!< The number of iterations
  long double alpha; //!<
  long double beta; //!<
  long double evaporationRate ;//!< the evaporation rate
  int Q ; //!< constant Q used in the pheromone update rule
  unsigned int numberOfAnts; //!<
  
  
};

#endif
