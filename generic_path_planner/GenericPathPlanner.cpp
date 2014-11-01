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

#include "GenericPathPlanner.h"
#include <math.h>
using namespace std;


GenericPathPlanner::GenericPathPlanner()
{
} 
GenericPathPlanner::~GenericPathPlanner(void){
}
GenericPathPlanner::GenericPathPlanner(int numIterations)
{
   setNumberOfIterations(numIterations);
}
void GenericPathPlanner::setNumberOfIterations (int numIterations)
{
  numberOfIterations=numIterations;
}
int GenericPathPlanner::getNumberOfIterations ()
{
  return numberOfIterations;
}
  /*******************************************************************************
 * Function Name: findFreeNeighborCell
 * Inputs: the row and columun of the current Cell
 * Output: a vector of free neighbor cells of the current cell
 * Description:it is used to find the free neighbors Cells of a the current Cell in the grid
 * Check Status: Checked by Anis, Imen and Sahar
*********************************************************************************/
vector <int> GenericPathPlanner::findFreeNeighborCell (OccupancyGridMap* map,int CellID){
 
  int rowID=map->getCellRowID(CellID);
  int colID=map->getCellColID(CellID);
 
  vector <int>  freeNeighborCells;

  for (int i=-1;i<=1;i++)
    for (int j=-1; j<=1;j++){
      //check whether the index is valid
     if ((rowID+i>=0)&&(rowID+i<map->getHeight())&&(colID+j>=0)&&(colID+j<map->getWidth()))
        if(map->isFree(rowID+i, colID+j) && (!(i==0 && j==0))){
	    freeNeighborCells.push_back(map->getCellIndex(rowID+i,colID+j));
	}
    }
    return  freeNeighborCells;
 
}

 /*******************************************************************************
 * Function Name: findfourFreeNeighborCell
 * Inputs: the row and columun of the current Cell
 * Output: a vector of free four neighbor cells of the current cell
 * Description:it is used to find the free four neighbors Cells of a the current Cell in the grid
 * Check Status: Checked by Anis, Imen 
*********************************************************************************/
vector <int> GenericPathPlanner::findfourFreeNeighborCell (OccupancyGridMap* map,int CellID){
 
  vector <int>  fourFreeNeighborCells;
 
  vector <int>  eightFreeNeighborCells=findFreeNeighborCell (map,CellID);
  
  for(uint i=0; i<eightFreeNeighborCells.size(); i++)
  {
    if(map->getMoveCost(CellID,eightFreeNeighborCells[i])==map->MOVE_COST)
      fourFreeNeighborCells.push_back(eightFreeNeighborCells[i]);
  }
  
    return  fourFreeNeighborCells;
 
}


/*******************************************************************************
 * Function Name: findNeighborCell
 * Inputs: the row and columun of the current Cell
 * Output: a vector of free neighbor cells of the current cell
 * Description:it is used to find the neighbors Cells of a the current Cell in the grid
 * Check Status: 
*********************************************************************************/
vector <int> GenericPathPlanner::findNeighborCell (OccupancyGridMap* map,int CellID){
 
  int rowID=map->getCellRowID(CellID);
  int colID=map->getCellColID(CellID);
 
  vector <int>  NeighborCells;

  for (int i=-1;i<=1;i++)
    for (int j=-1; j<=1;j++){
      //check whether the index is valid
     if ((rowID+i>=0)&&(rowID+i<map->getHeight())&&(colID+j>=0)&&(colID+j<map->getWidth()))
        if(!(i==0 && j==0)){
	    NeighborCells.push_back(map->getCellIndex(rowID+i,colID+j));
	}
    }

    return  NeighborCells;
 
}
  /*******************************************************************************
 * Function Name: isVisited
 * Inputs: the cellID and the path
 * Output: a boolean variable that indicates if the cellID exists in the path or not
 * Description:it is to verify if a cell exists in the path or not
 * Check Status: Checked by Anis, Imen and Sahar
*********************************************************************************/  
bool GenericPathPlanner::isVisited(int CellID, Path *path){
  
   for (uint i=0; i<path->getPath().size();i++) 
   {
	if(CellID == path->getPath()[i])
	  return true;
   }
	
	return false;
}
/*******************************************************************************
 * Function Name: getUnvisitedFreeNeighbors
 * Inputs: the neighbor cells and the path
 * Output: a vector of unvisited neighbor
 * Description:it is to remove the already visited neighbor from the path
 * Check Status: Checked by Anis, Imen and Sahar
*********************************************************************************/
vector <int> GenericPathPlanner::getUnvisitedFreeNeighbors (vector <int> neighborCells, Path *path){
  vector <int> unvisitedNeighborCells;
  for (uint i=0; i<neighborCells.size();i++)
      if (!isVisited(neighborCells[i], path))
	unvisitedNeighborCells.push_back(neighborCells[i]);
      
      return unvisitedNeighborCells;
}
/*******************************************************************************
 * Function Name: isDeadlock
 * Inputs: the current cell ID, the vector of unvisited neighbor cells and the 
   vector of Deadlock
 * Output: a boolean variable 
 * Description:it verify if the couple(cellID, neighbor) exists in the DeadLock vector or not
 * Check Status: Checked by Anis, Imen and Sahar
*********************************************************************************/
bool GenericPathPlanner::isDeadlock(int CellID, int unvisitedNeighborCell, vector <vector <int> > deadLockCells){
  
   for(uint j=0; j<deadLockCells.size(); j++)
	if((CellID == deadLockCells[j][0]) && (unvisitedNeighborCell==deadLockCells[j][1]))
	  return true;

	return false;
}
/*******************************************************************************
 * Function Name: getNonDeadlockFreeNeighbors
 * Inputs: the current cell ID, the vector of unvisited neighbor cells and the 
   vector of Deadlock
 * Output: a vector of neighbor cells 
 * Description:it used to search the neighbor cells that are non deadlock 
 * Check Status: Checked by Anis, Imen and Sahar
*********************************************************************************/
vector <int> GenericPathPlanner::getNonDeadlockFreeNeighbors (int CellID, vector <int> unvisitedNeighborCells, vector <vector <int> > deadLockCells){
  vector <int> neighborsNoDeadLock;
  for(uint i=0; i<unvisitedNeighborCells.size();i++)
       if (!isDeadlock(CellID, unvisitedNeighborCells[i], deadLockCells)){
	  neighborsNoDeadLock.push_back(unvisitedNeighborCells[i]);
       }
      return neighborsNoDeadLock;
}
/*******************************************************************************
 * Function Name: contains
 * Inputs: the cell ID, the vector of cells 
 * Output: true or false
 * Description:check if the cell exits in the vector
 * Check Status: Checked by Anis, Imen and Sahar
*********************************************************************************/
bool GenericPathPlanner::contains(int CellID, vector<int> vect){
   for (uint i=0; i<vect.size();i++) {
	if(CellID == vect[i])
	  return true;
   }
	return false;
}

/****************************************************/
// Function: areTwoPathsEqual
// To check if two paths are equal or not
// Input: two paths
// Output: return true if the two paths equal, false otherwise
/****************************************************/
bool GenericPathPlanner::areTwoPathsEqual(OccupancyGridMap* map, Path* path1, Path* path2) {

	if (path1->getPathCost(map, true) != path2->getPathCost(map, true))
		return false;
	else {
		if (path1->getPath().size() != path2->getPath().size())
			return false;
		else {
			for (uint i = 0; i < path1->getPath().size(); i++) {
				if (path1->getPath()[i] != path2->getPath()[i])
					return false;
			}
		}
	}
	return true;
}

/*******************************************************************************/
//Function Name: getValidNeighbors
//Inputs: the current Cell and the list of neighbors and the initial path
//Output: a vector of neighbor Cells that are not visited and that will not lead to a deadlock
//Description: 
/*********************************************************************************/
vector <int> GenericPathPlanner::getValidNeighbors(OccupancyGridMap* map,Path* path, int currentCell, vector <vector <int> > deadLockCells, int goalCell )
{
  vector<int> neighborCells;
  vector <int> unvisitedNeighborCells;
  vector <int> neighborsNoDeadLock;
    
  neighborCells= findFreeNeighborCell(map,currentCell); 
  //find unvisited neighbor cells
  unvisitedNeighborCells= getUnvisitedFreeNeighbors(neighborCells, path);
  //find non-deadlock neighbors
  neighborsNoDeadLock=getNonDeadlockFreeNeighbors(currentCell, unvisitedNeighborCells,deadLockCells);
   //find if the goal position is among the neighbors
  if(contains(goalCell, neighborsNoDeadLock)){
    neighborsNoDeadLock.clear();
    neighborsNoDeadLock.push_back(goalCell);    
  }
  return neighborsNoDeadLock;
}
/*******************************************************************************/
//Function Name: getBestNextCell
//Inputs: the current Cell and the list of neighbors
//Output: the next cell
//Description: it calculates the Move cost to the neighbors and return the best next cell
/*********************************************************************************/
int GenericPathPlanner::getBestNextCell(OccupancyGridMap* map,int currentCell, vector<int> neigborCells)
{
  int nextCell;
  float distanceCurrentToNeighbor;
  vector <float> distanceToNeighborsCells;
  float mindistance;
  int pos;

   for (uint i=0; i<neigborCells.size(); i++)
      {   
	distanceCurrentToNeighbor=0.0;	
	distanceCurrentToNeighbor=map->getMoveCost(map->getCellRowID(currentCell),map->getCellColID(currentCell),map->getCellRowID(neigborCells[i]),map->getCellColID(neigborCells[i]));
        distanceToNeighborsCells.push_back(distanceCurrentToNeighbor);
      }
      //search the minimum distance in the vector distanceToNeighborsCells
      mindistance = *min_element(distanceToNeighborsCells.begin(), distanceToNeighborsCells.end()); //minimum distance
      pos = find(distanceToNeighborsCells.begin(), distanceToNeighborsCells.end(), mindistance) - distanceToNeighborsCells.begin(); //index of the minimum distance in the vector 
      nextCell=neigborCells[pos];
   return nextCell;
}
/*******************************************************************************/
//Function Name: getBestNextCellSmartly
//Inputs: the current Cell, the list of neighbors, isSmart: if the neighbor cell will be choosen smartly or not
//Output: the next cell
//Description: it generates the next cell smartly if isSmart is true or by greedy method if isSmart is false
/*********************************************************************************/
int GenericPathPlanner::getBestNextCellSmartly(OccupancyGridMap* map, int currentCell, vector<int> neighborCells, int goal, bool isSmart)
{
  
  vector <float> distanceToNeighborsCells;
  int rowGoalCell=map->getCellRowID(goal);
  int colGoalCell=map->getCellColID(goal);
  
  int rowCurrentCell=map->getCellRowID(currentCell);
  int colCurrentCell=map->getCellColID(currentCell);
  
  if (isSmart){
  for (uint i =0; i<neighborCells.size(); i++)
  {
    int rowNeighborCell=map->getCellRowID(neighborCells[i]);
    int colNeighborCell=map->getCellColID(neighborCells[i]);
    if(rowGoalCell > rowCurrentCell && colGoalCell > colCurrentCell) 
    {
      if(rowNeighborCell > rowCurrentCell && colNeighborCell > colCurrentCell) //best solution
	return neighborCells[i];
     /* if(rowNeighborCell == rowCurrentCell && colNeighborCell > colCurrentCell) //other acceptable solution
	return neighborCells[i];
      if(rowNeighborCell > rowCurrentCell && colNeighborCell == colCurrentCell)  //other acceptable solution
	return neighborCells[i];
      if(rowNeighborCell < rowCurrentCell && colNeighborCell > colCurrentCell)  //less acceptable solution
	return neighborCells[i];
      if(rowNeighborCell > rowCurrentCell && colNeighborCell < colCurrentCell) //less acceptable solution
	return neighborCells[i];*/
    }
    else
    {
      if(rowGoalCell < rowCurrentCell && colGoalCell < colCurrentCell)
      {
	if(rowNeighborCell< rowCurrentCell && colNeighborCell < colCurrentCell) //best solution
	  return neighborCells[i];
	/* if(rowNeighborCell == rowCurrentCell && colNeighborCell < colCurrentCell) //other acceptable solution
	   return neighborCells[i];
         if(rowNeighborCell  < rowCurrentCell && colNeighborCell == colCurrentCell)  //other acceptable solution
	   return neighborCells[i];
         if(rowNeighborCell < rowCurrentCell && colNeighborCell > colCurrentCell)  //less acceptable solution
	   return neighborCells[i];
         if(rowNeighborCell > rowCurrentCell && colNeighborCell < colCurrentCell) //less acceptable solution
	  return neighborCells[i];*/
      }
      else
      {
	if(rowGoalCell > rowCurrentCell && colGoalCell < colCurrentCell)
	{
	  if(rowNeighborCell> rowCurrentCell && colNeighborCell < colCurrentCell)
	    return neighborCells[i];
	 /* if(rowNeighborCell == rowCurrentCell && colNeighborCell < colCurrentCell) //other acceptable solution
	   return neighborCells[i];
          if(rowNeighborCell  > rowCurrentCell && colNeighborCell == colCurrentCell)  //other acceptable solution
	   return neighborCells[i];
          if(rowNeighborCell > rowCurrentCell && colNeighborCell > colCurrentCell)  //less acceptable solution
	   return neighborCells[i];
          if(rowNeighborCell < rowCurrentCell && colNeighborCell < colCurrentCell) //less acceptable solution
	  return neighborCells[i];*/

	}
	else
	{
	  if(rowGoalCell < rowCurrentCell && colGoalCell > colCurrentCell)
	  {
	    if(rowNeighborCell< rowCurrentCell && colNeighborCell > colCurrentCell)
	      return neighborCells[i];
	  /*  if(rowNeighborCell == rowCurrentCell && colNeighborCell > colCurrentCell) //other acceptable solution
	      return neighborCells[i];
	    if(rowNeighborCell  < rowCurrentCell && colNeighborCell == colCurrentCell)  //other acceptable solution
	      return neighborCells[i];
	    if(rowNeighborCell > rowCurrentCell && colNeighborCell > colCurrentCell)  //less acceptable solution
	      return neighborCells[i];
	    if(rowNeighborCell < rowCurrentCell && colNeighborCell < colCurrentCell) //less acceptable solution
	      return neighborCells[i];*/
	  } 
	    else
	    {
	      if(rowGoalCell == rowCurrentCell && colGoalCell > colCurrentCell) 
	      {
		if(rowNeighborCell == rowCurrentCell && colNeighborCell > colCurrentCell) 
		  return neighborCells[i];
		/*if(rowNeighborCell < rowCurrentCell && colNeighborCell > colCurrentCell) //other acceptable solution
	          return neighborCells[i];
	       if(rowNeighborCell  > rowCurrentCell && colNeighborCell > colCurrentCell)  //other acceptable solution
	          return neighborCells[i];
	       if(rowNeighborCell >  rowCurrentCell && colNeighborCell == colCurrentCell)  //less acceptable solution
	          return neighborCells[i];
	       if(rowNeighborCell < rowCurrentCell && colNeighborCell == colCurrentCell) //less acceptable solution
	          return neighborCells[i];     
	       */
		
	      }
	      else
	      {
		if(rowGoalCell == rowCurrentCell && colGoalCell < colCurrentCell)
		{
		      if(rowNeighborCell== rowCurrentCell && colNeighborCell < colCurrentCell)
			return neighborCells[i];
		   /*   if(rowNeighborCell < rowCurrentCell && colNeighborCell < colCurrentCell) //other acceptable solution
	                return neighborCells[i];
	             if(rowNeighborCell  > rowCurrentCell && colNeighborCell < colCurrentCell)  //other acceptable solution
	                return neighborCells[i];
	             if(rowNeighborCell >  rowCurrentCell && colNeighborCell == colCurrentCell)  //less acceptable solution
	               return neighborCells[i];
	             if(rowNeighborCell < rowCurrentCell && colNeighborCell == colCurrentCell) //less acceptable solution
	               return neighborCells[i];*/
		}
		else
		{
		    if(rowGoalCell > rowCurrentCell && colGoalCell == colCurrentCell)
		    {
			    if(rowNeighborCell> rowCurrentCell && colNeighborCell == colCurrentCell)
			      return neighborCells[i];
			 /*   if(rowNeighborCell > rowCurrentCell && colNeighborCell < colCurrentCell) //other acceptable solution
			    return neighborCells[i];
			    if(rowNeighborCell  > rowCurrentCell && colNeighborCell > colCurrentCell)  //other acceptable solution
			    return neighborCells[i];
			    if(rowNeighborCell ==  rowCurrentCell && colNeighborCell < colCurrentCell)  //less acceptable solution
			    return neighborCells[i];
			    if(rowNeighborCell == rowCurrentCell && colNeighborCell > colCurrentCell) //less acceptable solution
			    return neighborCells[i];*/
		    }
		    else
		    {
			    if(rowGoalCell < rowCurrentCell && colGoalCell == colCurrentCell)
			    {
			      if(rowNeighborCell< rowCurrentCell && colNeighborCell == colCurrentCell)
				return neighborCells[i];
			    /*  if(rowNeighborCell < rowCurrentCell && colNeighborCell > colCurrentCell) //other acceptable solution
				return neighborCells[i];
			      if(rowNeighborCell  < rowCurrentCell && colNeighborCell < colCurrentCell)  //other acceptable solution
				return neighborCells[i];
			      if(rowNeighborCell == rowCurrentCell && colNeighborCell < colCurrentCell)  //less acceptable solution
				return neighborCells[i];
			      if(rowNeighborCell == rowCurrentCell && colNeighborCell > colCurrentCell) //less acceptable solution
				return neighborCells[i];*/
				
			    }
		      }
		  }
	      }
	    }  
      }
      }
      }
    }
 //return getBestNextCell(map,currentCell,neighborCells);
  int pos=rand()%(neighborCells.size());
  return neighborCells[pos];
  }
  else
  return getBestNextCell(map,currentCell,neighborCells);
}
/*******************************************************************************/
//Function Name: getBestNextCellEuclidian
//Inputs: the map, the list of neighbors, the goal cell
//Output: the next cell
//Description: it generates the next cell using Euclidian distance
/*********************************************************************************/
int GenericPathPlanner::getBestNextCellEuclidian(OccupancyGridMap* map, vector<int> neighborCells, int goalCell)
{
  vector<float>distancesToGoal;
  for(int k=0; k<neighborCells.size(); k++)
  {
    distancesToGoal.push_back(sqrt((pow((map->getCellRowID(neighborCells[k])-map->getCellRowID(goalCell)),2))+pow((map->getCellColID(neighborCells[k])-map->getCellColID(goalCell)), 2)));
  }
   float mindistance = *min_element(distancesToGoal.begin(), distancesToGoal.end());
   int pos = find(distancesToGoal.begin(), distancesToGoal.end(), mindistance) - distancesToGoal.begin();
   
   return neighborCells[pos];
   
}
/*******************************************************************************/
//Function Name: getBestNextCellManhattan
//Inputs: the map, the list of neighbors, the goal cell
//Output: the next cell
//Description: it generates the next cell using Manhattan distance
/*********************************************************************************/
int GenericPathPlanner::getBestNextCellManhattan(OccupancyGridMap* map, vector<int> neighborCells, int goalCell)
{
  vector<float>distancesToGoal;
  for(int k=0; k<neighborCells.size(); k++)
  {
    distancesToGoal.push_back(abs((map->getCellRowID(neighborCells[k])-map->getCellRowID(goalCell)))+ abs(map->getCellColID(neighborCells[k])-map->getCellColID(goalCell)));
  }
   float mindistance = *min_element(distancesToGoal.begin(), distancesToGoal.end());
   int pos = find(distancesToGoal.begin(), distancesToGoal.end(), mindistance) - distancesToGoal.begin();
   
   return neighborCells[pos];
   
}
/*******************************************************************************/
//Function Name: backtracking
//Inputs: the path, the goal position and the vector of deadlock positions
//Output: the next Cell
//Description: it generates the next cell by backtracking.
/*********************************************************************************/
int GenericPathPlanner::backtracking(OccupancyGridMap* map, vector <vector <int> > &deadLockPos, Path *& path, int goalCell)
{
   bool neighborsFound;
   vector <int> couple_PreviousCell_DeadLock;//it contains one couple (previousCell, DeadlockCell) 
   int currentCell;
   vector <int> neighborsCellsAfterBacktracking; 
   neighborsFound=false;
   
      while(neighborsFound==false) //backtraking until find a cell that has neighbors
      {
	couple_PreviousCell_DeadLock.clear();
	uint i=0; 
	while (i<deadLockPos.size())
	{
	  if(path->getPath()[path->getPath().size()-2] == deadLockPos[i][0] && path->getPath()[path->getPath().size()-1]==deadLockPos[i][1])
	  {
	    break;
	  }
	  i=i+1;
	}
	if(i==deadLockPos.size()) // put the couple (previous, deadlock) in the vector deadLockPos
	{
	  couple_PreviousCell_DeadLock.push_back(path->getPath()[path->getPath().size()-2]);
	  couple_PreviousCell_DeadLock.push_back(path->getPath()[path->getPath().size()-1]);	  
	  deadLockPos.push_back(couple_PreviousCell_DeadLock);
	}
	path->removeCell(map,path->getPath().size()-1);//remove the last cell from the vector

	if (path->getPath().size() == 0){
		return -1;
	}

	currentCell= path->getPath()[path->getPath().size()-1];
	neighborsCellsAfterBacktracking.clear();
	neighborsCellsAfterBacktracking=getValidNeighbors(map,path,currentCell,deadLockPos,goalCell); //search the neighbors of the current cell
	//if the size of the  vector of neighbor is superior to 0
	if (neighborsCellsAfterBacktracking.size()>0)
	{
	  neighborsFound=true;	 
	  currentCell=getBestNextCellEuclidian(map,neighborsCellsAfterBacktracking,goalCell);
	  //currentCell=getBestNextCellManhattan(map,neighborsCellsAfterBacktracking,goalCell);

	}
      }
   return currentCell;
}
/*******************************************************************************/
//Function Name: isStartAndGoalCellsValid
//Inputs: the start and Goal cells
//Output: true if the start and the goal cells are valid
//Description: check if the start and goal cells are valid
/*********************************************************************************/
bool GenericPathPlanner::isStartAndGoalCellsValid(OccupancyGridMap* map,int startCell,int goalCell)
{ 
 bool isvalid=true;
 bool isFreeStartCell=map->isFree(map->getCellRowID(startCell),map->getCellColID(startCell));
 bool isFreeGoalCell=map->isFree(map->getCellRowID(goalCell),map->getCellColID(goalCell));
    if (startCell==goalCell)
    {
    //cout << "The Start and the Goal cells are the same..." << endl; 
    isvalid = false;
    }
   else
   {
      if (!isFreeStartCell && !isFreeGoalCell)
      {
	//cout << "The start and the goal cells are obstacle positions..." << endl;
        isvalid = false;
      }
      else
      {
	if (!isFreeStartCell)
	{
	  //cout << "The start is an obstacle..." << endl;
	  isvalid = false;
	}
	else
	{
	    if(!isFreeGoalCell)
	    {
	      //cout << "The goal cell is an obstacle..." << endl;
	      isvalid = false;
	    }
	    else
	    {
	      if (findFreeNeighborCell(map,goalCell).size()==0)
	      {
		//cout << "The goal cell is encountred by obstacles... "<< endl;
		isvalid = false;
	      }
	      else
	      {
		if(findFreeNeighborCell(map,startCell).size()==0)
		{
		  //cout << "The start cell is encountred by obstacles... "<< endl;
		  isvalid = false;
		}
	      }
	    }
	}
      }
  }
 return isvalid;
}
/*******************************************************************************/
//Function Name: GenerateInitialSolution
//Inputs: the start and the goal cells
//Output: the initial path
//Description: it generates the initial path
/*********************************************************************************/
Path* GenericPathPlanner::getInitialPath(OccupancyGridMap* map,int startCell, int goalCell, int isSmart)
{
	  Path* emptyPath=new Path();

if (isStartAndGoalCellsValid(map, startCell, goalCell)){

  ofstream file ("generic", ios::trunc) ;
  Path* initialPath;// the initial path that will be generated
  vector <vector <int> > deadLockPos; // it contains the couples of (previousCell, DeadlockCell).
  vector <int> neighborCells;
  int currentCell; // the current cell

 	      initialPath=new Path();
	      initialPath->setCost(0.0);
	      initialPath->insertCell(map,0,startCell);
	      currentCell=startCell;

	      // while the goal position has not reached.

	      while ((currentCell!=goalCell) && (currentCell != -1))
	      {
		neighborCells.clear();
		neighborCells=getValidNeighbors(map,initialPath,currentCell,deadLockPos,goalCell); //search the neighbor cells

		//2 cases: 
		if(neighborCells.size()>0){ //if the size of the neighbors vector is superior to 0
			currentCell=getBestNextCellEuclidian(map,neighborCells,goalCell); //choose the nextCell
			//currentCell=getBestNextCellManhattan(map,neighborCells,goalCell);
		}
		else{ //if the size of the vector of neighboor is equal to 0: we fall in a deadlock
		  currentCell=backtracking(map,deadLockPos, initialPath, goalCell);
		}

		if (currentCell == -1){
			return emptyPath;
		}

		initialPath->insertCell(map,initialPath->getPath().size(),currentCell);	
	      }

  return initialPath;

}
	else {
		cout<<"NOT valid start or goal";
		return emptyPath;
		//exit(1);
	}

}
/*******************************************************************************/
//Function Name: drawStraightLineStartGoal
//Inputs: the map layout, the goal, the start positions 
//Output: an initial path (feasible or unfeasible)
//Description: it is used to draw a staright line between the start and goal positions
/*********************************************************************************/
Path* GenericPathPlanner::drawStraightLineStartGoal(OccupancyGridMap* OGM,int startCell, int goalCell)
{
        int i1=OGM->getCellRowID(startCell);
        int j1=OGM->getCellColID(startCell);
        int i2=OGM->getCellRowID(goalCell);
        int j2=OGM->getCellColID(goalCell);

	 // Add start cell to the path
	Path* initialPath=new Path();
	initialPath->setCost(0.0);
        initialPath->insertCell(OGM,0,startCell);
        
	/*************************************************************/
	// In case start & goal cell in the same row - move horizontally
	if (i1 == i2) {

	    if (j1 < j2) // move to the right
		   for (int k = j1 + 1; k <= j2; k++) // add all the cells between the start and goal
	              initialPath->insertCell(OGM,initialPath->getPath().size(),OGM->getCellIndex(i1,k));
		else //move to the left
		  for (int k = j1 - 1; k >= j2; k--) // add all the cells between the start and goal
		      initialPath->insertCell(OGM,initialPath->getPath().size(),OGM->getCellIndex(i1,k));
	}
	/*************************************************************/
	// in case start & goal cell in the same column - move vertically
	else {
		if (j1 == j2) {
			if (i1 < i2) // move down
				for (int k = i1 + 1; k <= i2; k++) // add all the cells between the start and goal
				  initialPath->insertCell(OGM,initialPath->getPath().size(),OGM->getCellIndex(k,j1));
			
			else //move up
			  for (int k = i1 - 1; k >= i2; k--) // add all the cells between the start and goal	
			    initialPath->insertCell(OGM,initialPath->getPath().size(),OGM->getCellIndex(k,j1));		
		}
		/*************************************************************/
		// In case move diagonally to the up
		else {
			if (i1 > i2) {
				if (j1 > j2) // diagonally (left + up)
                               {
					while (j1 != j2 && i1 != i2) {
					    i1--;
					    j1--;
				           initialPath->insertCell(OGM,initialPath->getPath().size(),OGM->getCellIndex(i1,j1));		
					}

					if (i1 == i2) // Then horizontally to left	
						for (int k = j1 - 1; k >= j2; k--) // add all the cells between the start and goal	
							initialPath->insertCell(OGM,initialPath->getPath().size(),OGM->getCellIndex(i1,k));
						
					else //  or vertically to up
						for (int k = i1 - 1; k >= i2; k--) // add all the cells between the start and goal
							initialPath->insertCell(OGM,initialPath->getPath().size(),OGM->getCellIndex(k,j1));
                                }
				else //diagonally (right + up)
                               {
				  while (j1 != j2 && i1 != i2) {
					i1--;
					j1++;	
					initialPath->insertCell(OGM,initialPath->getPath().size(),OGM->getCellIndex(i1,j1));	
					}
					
					if (i1 == i2) // Then horizontally to right	
					  for (int k = j1 + 1; k <= j2; k++) // add all the cells between the start and goal		
							initialPath->insertCell(OGM,initialPath->getPath().size(),OGM->getCellIndex(i1,k));	
					else //  or vertically to up
					  for (int k = i1 - 1; k >= i2; k--) // add all the cells between the start and goal
						initialPath->insertCell(OGM,initialPath->getPath().size(),OGM->getCellIndex(k,j1));	
                                }
			/*************************************************************/
			// In case move diagonally to the down
			} else //move diagonally and down
			{
				if (j1 > j2) // diagonally (left + down)
                                {
					while (j1 != j2 && i1 != i2) {
					   i1++;
					   j1--;
					   initialPath->insertCell(OGM,initialPath->getPath().size(),OGM->getCellIndex(i1,j1));
					}
					
					if (i1 == i2) // Then horizontally to left
					  for (int k = j1 - 1; k >= j2; k--) // add all the cells between the start and goal
							initialPath->insertCell(OGM,initialPath->getPath().size(),OGM->getCellIndex(i1,k));
					else //  or vertically to down
					  for (int k = i1 + 1; k <= i2; k++) // add all the cells between the start and goal	
					     initialPath->insertCell(OGM,initialPath->getPath().size(),OGM->getCellIndex(k,j1));
                                }
				else //diagonally (right + down)
                               {
				  while (j1 != j2 && i1 != i2) {
					 i1++;
					 j1++;
					 initialPath->insertCell(OGM,initialPath->getPath().size(),OGM->getCellIndex(i1,j1));
					}
					if (i1 == i2) // Then horizontally to right
						for (int k = j1 + 1; k <= j2; k++) // add all the cells between the start and goal	
							initialPath->insertCell(OGM,initialPath->getPath().size(),OGM->getCellIndex(i1,k));
					else //  or vertically to down
				         for (int k = i1 + 1; k <= i2; k++) // add all the cells between the start and goal	
							initialPath->insertCell(OGM,initialPath->getPath().size(),OGM->getCellIndex(k,j1));	
                               }
			}
		}
	}
	initialPath->insertCell(OGM,initialPath->getPath().size(),goalCell);
	
	/*cout << "The straight path is: " << endl;
	for (uint i=0; i< initialPath->getPath().size(); i++)
	{
	  cout << initialPath->getPath()[i] << "\t";
	}
	cout << endl;
	bool feasible=initialPath->isFeasible(OGM);
	cout << "feasible= " << feasible << endl;*/
	return initialPath;
}
/*******************************************************************************/
//Function Name: getNeighborsAtRadiusN
//Inputs: the map layout, the radius n and the CellID
//Output: a vector of neighbors at radius N
//Description: 
/*********************************************************************************/
vector <int> GenericPathPlanner:: getNeighborsAtRadiusN(OccupancyGridMap* map,int radius,int CellID)
{      
       vector <int> neighborsRadiusN=findNeighborCell(map,CellID);  
       int size=neighborsRadiusN.size();
       if(radius>1)
       {
	 int h=1;
         int i=0;
	  while(i<size)
	    {
	      vector<int> neigborsofNeigbors=findNeighborCell(map,neighborsRadiusN[i]);
	      for(uint j=0; j<neigborsofNeigbors.size(); j++)
	      {
		  //search if the neighbor already exist in the vector
		  int pos = find(neighborsRadiusN.begin(), neighborsRadiusN.end(), neigborsofNeigbors[j]) - neighborsRadiusN.begin();
		  if(pos>=neighborsRadiusN.size() && neigborsofNeigbors[j] != CellID)
		  neighborsRadiusN.push_back(neigborsofNeigbors[j]);
	      }
	      i++;
	      if(i==size)
	      {
		size=neighborsRadiusN.size();
	        h++;
	      }
	      if(h==radius)
		break;
	    } 
       }		
	//remove all the obstacles from the vector
	vector <int> freeNeighborsRadiusN;
	for(uint i=0;i<neighborsRadiusN.size(); i++)
	{
	  if(map->isFree(neighborsRadiusN[i]))
	    freeNeighborsRadiusN.push_back(neighborsRadiusN[i]);
	}	
	return freeNeighborsRadiusN;
}
/*******************************************************************************/
//Function Name: findPath
//Inputs: the map and the initial path
//Output: the robot path
//Description: it generates the robot path
/*********************************************************************************/
Path* GenericPathPlanner::findPath(OccupancyGridMap* map,Path* initialPath)
{
  return initialPath;
}
/*******************************************************************************/
//Function Name: findPath
//Inputs: the maps, the start and the goal cells
//Output: the robot path
//Description: it generates the robot path
/*********************************************************************************/
Path* GenericPathPlanner::findPath(OccupancyGridMap* map,int startCell, int goalCell)
{
  return 0;
}
Path* GenericPathPlanner::findPath(OccupancyGridMap* map,int startCell, int goalCell,bool withBreakTies,int neighborNumber)
{
  return 0;
}
Path* GenericPathPlanner::findPath(OccupancyGridMap* map,Path* initialPath, timespec & time)
{
  return 0;
}

