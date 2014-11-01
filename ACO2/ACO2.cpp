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


#include"ACO.h"
#include <vector>
#include <float.h>

using namespace std;
ofstream file100 ("traceFile.txt", ios::trunc) ;

/**********************************************************/
//Constructors and destructors
/**********************************************************/
ACO::ACO(void)
{
}
ACO::ACO(float** PM)
{
   setPheromoneMatrix(PM);
}
ACO::ACO(float initPheromoneValue ,float alpha , float beta , float evaporationRate ,int Q , unsigned int numberOfAnts,int numberOfIterations):GenericACO(initPheromoneValue,alpha,beta,evaporationRate,Q,numberOfAnts, numberOfIterations)
{  
}
ACO::~ACO(void)
{
}
/**********************************************************/
//Function: Mutators
/**********************************************************/
void ACO::setPheromoneMatrix (float** PM)
{
	pheromoneMatrix=PM;
}
/**********************************************************/
//Function: Accessors
/**********************************************************/
float**  ACO::getPheromoneMatrix()
{
	return pheromoneMatrix;
}
/**********************************************************/
// Function initializePheromoneMatrix
// it used to initialze the Matrix of pheromone 
// Input: the occupancy grid map
/**********************************************************/
void ACO::initializePheromoneMatrix(OccupancyGridMap* map)
{ 
  pheromoneMatrix = (float**) malloc(sizeof( float*)*map->getHeight());
  for(int i=0;i<map->getWidth();i++)
  pheromoneMatrix[i] = (float*) malloc(sizeof(float)*map->getWidth());
 
  for (int i=0; i< map->getWidth(); i++)
   {
     for(int j=0; j<map->getHeight(); j++)
     {
       if (map->isFree(i,j)) 
       { 
	  getPheromoneMatrix()[i][j]=getInitialPheromoneValue();
       }
       else
	  getPheromoneMatrix()[i][j]=FLT_MAX;
     }
   } 
}
/**********************************************************/
// Function computeTransitionProbabilities
// it calculates the transition probabilities of the ants between the cells
// Input: the map, the current Cell and the Unvisited Neighbor Cells of the current cell.
// Output: the probabilities calculated by the transition rule probability.
/**********************************************************/
vector <float> ACO::computeTransitionProbabilities(OccupancyGridMap* map,int currentCell,vector <int> unvisitedNeighborCells)
{
        vector <float> probabilities; //it contains the probabilities of transition
	float etha; // the inverse of euclidian distance between the current cell and the neighbor cell
	
	//calculate the probabilities using the transition rule probability
	float sum;
	for (uint i=0; i<unvisitedNeighborCells.size() ; i++)
	{
	      etha = 1/map->getMoveCost(currentCell,unvisitedNeighborCells[i]);	 
	      probabilities.push_back(pow(getPheromoneMatrix()[map->getCellRowID(unvisitedNeighborCells[i])][map->getCellColID(unvisitedNeighborCells[i])],getAlpha()) * pow(etha,getBeta()));
        }
        
        sum = std::accumulate(probabilities.begin(),probabilities.end(),0.0);
	if (sum != 0)     
	  for (uint i=0; i<unvisitedNeighborCells.size() ; i++)
	    {
		probabilities[i] = probabilities[i]/sum ;
	    }
	
	return probabilities;
}
/**********************************************************/
// Function getNextCell
// it is used to select the next cell of the ant.
// input: a vector of neighbor cells and their probabilities.
// output: the next cell of the ant.
//Check Status: Checked by Imen 
/**********************************************************/
int ACO::getNextCell(vector <int> NeighborCells, vector < float> probabilities)
{
  int nextCell;
  vector < float> cumulativeProbabilities;
  float randomNumber;
  
  //calculate the cumulative probabilities
  cumulativeProbabilities.push_back(probabilities[0]);
  for(uint i=1; i<probabilities.size();i++)
  {
    cumulativeProbabilities.push_back(cumulativeProbabilities[i-1]+probabilities[i]);
  }
	     
  //choose a random number
  randomNumber =1-((float)rand() / (float)RAND_MAX);
  
 
  // choose the next cell
    //case1:
    if (randomNumber >= 0 && randomNumber <= cumulativeProbabilities[0])
    {
      nextCell=NeighborCells[0];
    }
    else
    {
        //case2:
        if(randomNumber >= cumulativeProbabilities[cumulativeProbabilities.size()-1] && randomNumber <= 1)
        {
           nextCell=NeighborCells[NeighborCells.size()-1];
	}
	//case3:
        else
	{
	     for(int i=0; i< cumulativeProbabilities.size(); i++)
	     {
	        if(randomNumber >= cumulativeProbabilities[i] &&  randomNumber <= cumulativeProbabilities[i+1])
		{
                   float difference1=randomNumber- cumulativeProbabilities[i];
                   float difference2=cumulativeProbabilities[i+1]- randomNumber;
                   if (difference1< difference2) 
		      nextCell=NeighborCells[i];
                   else
		      nextCell=NeighborCells[i+1];
		}

	     }
	}
     }
   return  nextCell ;
}
/**********************************************************/
// Function updatePheromone
// it is used to update the quantity of pheromone.
// input: vector of Ant that include the ants id, their paths and their paths cost.
// output:
/**********************************************************/
void ACO::updatePheromone(OccupancyGridMap* map,vector <Ant*> ants)
{  
   float delta; // the quantity of pheromone laid on edge (cell1, cell2) joining two cells i and j by all ants
   if(ants.size()!=0)
   {
	for(uint i=0; i<map->getHeight();i++)
	{
	  for(uint j=0; j<map->getWidth(); j++)
	  {  
	   if(getPheromoneMatrix()[i][j]!=FLT_MAX)
	   {
	     delta=getDelta(ants,map->getCellIndex(i,j));
	     getPheromoneMatrix()[i][j]=getPheromoneMatrix()[i][j] * (1-getEvaporationRate()) + delta;
	   }
	  }
	}
      }	
}
/**********************************************************/
// Function getDelta
// iIt is used to calculate the delta value used in the update pheromone rule.
// Input: list of ants the index of the cells i and j.
// Output: the delta value.
//Check Status: Checked by Imen 
/**********************************************************/
float ACO::getDelta(vector <Ant*> Ants, int cell)
{
  float delta = 0; 
  float Lk =0; // cost of the constructed path by all the ants .
  bool exist = false;
  for (uint k = 0; k < Ants.size(); k++)  
   {
      if(Ants[k]->getConstructedPath()->getCost()!=0)
      {
	    for (uint l = 0; l< Ants[k]->getConstructedPath()->getPath().size(); l++)
	    {
	    // if the edge relying two cells cell1 and cell2
	      if ( Ants[k]->getConstructedPath()->getPath()[l] == cell) 
		{
		    // if the ant k uses the position cell
		    // the length of the path constructed by the ant k is added to Lk value.
		    Lk = Lk + Ants[k]->getConstructedPath()->getCost();
		    exist=true;
		    break;
		}
	    }
      }
     }
	if (exist == true) 
	{   
		// if the edge (cell1,cell2) is used by the ants then the value of delta is different from 0
		//thus the quantity of pheromone will increase else it will evaporate
		delta = getQ()/Lk;
	}
 
	return delta;
}
/*******************************************************************************/
//Function Name: findPath
//Inputs: the map, the start cell and the goal cell
//Output: the best path of the algorithm
//Description: it is used to generate the best path after N iterations 
//Check Status: Checked by Imen 
/*********************************************************************************/
Path* ACO::findPath(OccupancyGridMap* map, int startCell, int goalCell, bool enable_trace)
{
 Path* bestPath=new Path(); //the best path
 vector<Path*> bestPaths; // it contains the best paths generated after each iteration
 vector <int> neighborCell; //the vector of neighbor cells
 vector <int> unvisitedFreeNeighborCell; //the unvisited neighbor cells
 int currentCell; //the current  position of the ant
 int nextCell; //the new position  of the ant
 vector <float> probabilities ; // it contains the probabilities of the possible next positions of an ant
 vector<Ant*> Ants(getNumberOfAnts()); // vector of ant
 int indexBestPath; // index of the best path of the algorithm
 
 
 //create the ants
 for(uint j=0 ; j<getNumberOfAnts();j++)
      Ants[j]=new Ant();
      
 //intialize the pheromone matrix
 initializePheromoneMatrix(map);

 for ( int i=0; i< getNumberOfIterations() ; i++) //ACO is repeated "numberOfIterations" times
   {
     cout << "************Itertion N° " << i << "****************" << endl;
     for(uint j=0 ; j<getNumberOfAnts();j++) // for each Ant
     {
      currentCell=startCell;
      Path* pathAnt=new Path();
      Ants[j]->setPosition(currentCell);
      pathAnt->insertCell(map,0,currentCell);
      Ants[j]->setConstructedPath(pathAnt);
     int index=1;
      //while the ant has not reached the goal position or not blocked
      while  (currentCell != goalCell)
      {  
         neighborCell=findFreeNeighborCell(map,currentCell);
         unvisitedFreeNeighborCell=getUnvisitedFreeNeighbors(neighborCell,pathAnt);
	  
         if (unvisitedFreeNeighborCell.size() == 0)// if the ant is blocked and there is no unvisited neighbor cells so the ant will be discarted 
           {
              pathAnt->getPath().clear();
	      Ants[j]->setConstructedPath(pathAnt);
	      Ants[j]->getConstructedPath()->setCost(0);
	      break;
	    }
	 else
	   {     
	      probabilities=computeTransitionProbabilities(map,currentCell,unvisitedFreeNeighborCell);
	      nextCell=getNextCell(unvisitedFreeNeighborCell,probabilities);
	      currentCell = nextCell;
	      pathAnt->insertCell(map,index,nextCell);	      
	      index++;
	   
	      if (currentCell == goalCell) // if the ant has reached the goal cell
	        { 
		  Ants[j]->setConstructedPath(pathAnt);
		  Ants[j]->getConstructedPath()->setCost(pathAnt->getPathCost(map,false)); 		
		 } 
	    }
       }
       pathAnt->getPath().clear(); 
     }
 
 // The best path after one iteration
     Path* bestPathInAnIteration=new Path();
     bestPathInAnIteration= findBestAnt(map,Ants);
      
     if(bestPathInAnIteration->getCost()!=0)
     {
	cout<<"The best path after iteration " << i << endl;
	bestPathInAnIteration->printPath();
	cout << "Its cost is " <<bestPathInAnIteration->getPathCost(map,false) << endl;
	bestPaths.push_back(bestPathInAnIteration);
     }
   // update the pheromone list
   updatePheromone(map,Ants);
    
  }
  
  // The best path generated by the algorithm
  if(bestPaths.size()!=0)
  {  
     indexBestPath=findBestPath(map,bestPaths); 
     bestPath->setPath(bestPaths[indexBestPath]->getPath());
     bestPath->setCost(bestPaths[indexBestPath]->getPathCost(map,false));
    
  }
 
 return bestPath ;
}
















