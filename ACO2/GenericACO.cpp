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


#include "GenericACO.h"
#include <vector>

using namespace std;

/**********************************************************/
//Constructors and destructors
/**********************************************************/
GenericACO::GenericACO(void)
{
}
GenericACO::GenericACO(long double initPheromoneValue ,long double alpha , long double beta , long double evaporationRate ,int Q , unsigned int numberOfAnts,int numberOfIterations)
{  
  setInitialPheromoneValue(initPheromoneValue);
  setAlpha(alpha);
  setBeta(beta);
  setEvaporationRate(evaporationRate);
  setQ(Q);
  setNumberOfAnts(numberOfAnts);
  setNumberOfIterations(numberOfIterations);
}
GenericACO::~GenericACO(void)
{
}

/**********************************************************/
//Function: Mutators
/**********************************************************/
void GenericACO::setAlpha(long double Alpha)
{
	alpha=Alpha;
}
void GenericACO::setBeta(long double Beta)
{
	beta= Beta;
}
void GenericACO::setEvaporationRate(long double EvaporationRate)
{
	evaporationRate=EvaporationRate;
}
void GenericACO::setQ(int q)
{
	Q=q;
}
void GenericACO::setNumberOfAnts(unsigned int NumberOfAnts)
{
	numberOfAnts=NumberOfAnts;
}
void GenericACO::setInitialPheromoneValue(long double InitialPheromoneValue)
{
	initialPheromoneValue=InitialPheromoneValue;
}
/**********************************************************/
//Function: Accessors
/********************************************************/
long double GenericACO::getAlpha()
{
	return alpha;
}
long double GenericACO::getBeta()
{
	return beta;
}
long double GenericACO::getEvaporationRate()
{
	return evaporationRate;
}
int GenericACO::getQ()
{
	return Q;
}
unsigned int GenericACO::getNumberOfAnts()
{
	return numberOfAnts;
}
long double GenericACO::getInitialPheromoneValue()
{
	return initialPheromoneValue;
}
/*******************************************************************************/
//Function Name: findBestPath
//Inputs: the map, a vector of paths
//Output: index of the best path in the vector
//Description: it is to generate the best path from a set of candidate paths 
/*********************************************************************************/
int GenericACO::findBestPath(OccupancyGridMap* map, vector<Path*> paths)
{ 
  
  int indexBestPath=0;
  float minPathCost=paths[0]->getPathCost(map,false);
  for (uint i=0; i<paths.size(); i++)
  { 
    if(paths[i]->getPathCost(map,false)<= minPathCost)
    { 
      minPathCost=paths[i]->getPathCost(map,false);
      indexBestPath=i;
    }
    
  }
  return indexBestPath;
}
/*******************************************************************************/
//Function Name: findBestAnt
//Inputs: the map, a vector of Ant
//Output: index of the best Ant in the vector
//Description: it is to generate the best ant from a set of candidate ants 
/*********************************************************************************/
Path* GenericACO::findBestAnt(OccupancyGridMap* map, vector<Ant*> Ants)
{
   Path* bestPathOfTheAnt=new Path();//best path of the ant

  int indexBestAnt=0; // index of the best ant in the vector Ants
  float minPathCost=0; //minimum path cost
  
  //initialize the minimum path cost
  for (uint i=0; i<Ants.size(); i++)
  {
    if(Ants[i]->getConstructedPath()->getPathCost(map,false)!=0)
    {
      minPathCost=Ants[i]->getConstructedPath()->getPathCost(map,false);
      break;
    }
  }
  //identify the minimum path cost in the vector ants
  if(minPathCost!=0)
  {
    for (uint i=0; i<Ants.size(); i++)
    { 
	if(Ants[i]->getConstructedPath()->getPathCost(map,false)<= minPathCost && Ants[i]->getConstructedPath()->getPathCost(map,false)!=0)
	    { 
		minPathCost=Ants[i]->getConstructedPath()->getPathCost(map,false);
		indexBestAnt=i;
	    }   
    }
    bestPathOfTheAnt=Ants[indexBestAnt]->getConstructedPath();
  }
  
  return bestPathOfTheAnt;
}
