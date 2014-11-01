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


#include "RAStar.h"
#include<iostream>
#include<fstream>
#include <stdio.h>
#include <algorithm> 
#include <list>
#include <math.h>
#include <limits>

using namespace std;


/**********************************************************/
//Function: Constructor
/**********************************************************/
RAStar::RAStar(int NumberOfNeighbors,bool BT)
{
  setNeighborNumber(NumberOfNeighbors);
  setIsWithBreakTies(BT);
}
RAStar::RAStar()
{
  list<cells> OPL; 
  setOpenList(OPL);
}
/**********************************************************/
//Function: destructor
/**********************************************************/
RAStar::~RAStar(void){
}
/**********************************************************/
//Function: Mutators
/**********************************************************/
void RAStar::setOpenList(list<cells> OPL){
  openList=OPL;
}
void RAStar::setGScore(float** gScore)
{
  g_score=gScore;
}
void RAStar::setNeighborNumber(int neighbor)
{
  neighborNumber=neighbor;
}
void RAStar::setIsWithBreakTies(bool withBT)
{
  isWithBreakTies=withBT;
}
/**********************************************************/
//Function: Accessors
/**********************************************************/
list <cells> RAStar::getOpenList(){
   return openList;
 }
float** RAStar::getGScore()
{
  return g_score;
}
int RAStar::getNeighborNumber()
{
  return neighborNumber;
}
bool RAStar::getIsWithBreakTies()
{
  return isWithBreakTies;
}
/*******************************************************************************/
//Function Name: addNeighborCellToOpenList
//Inputs: the open list, the neighbors Cell, the g_score matrix, the goal cell 
//Output: 
//Description: it is used to add a neighbor Cell to the open list
/*********************************************************************************/
void RAStar::addNeighborCellToOpenList(OccupancyGridMap* map, list<cells> & OPL, int neighborCell, float** g_score,int goalCell, float tBreak)
{
    cells CP;
    CP.currentCell=neighborCell; //insert the neighbor cell
    //calculate fcost
    if (getIsWithBreakTies()==true)
      CP.fCost=g_score[map->getCellRowID(neighborCell)][map->getCellColID(neighborCell)]+tBreak*calculateHCost(map,neighborCell,goalCell);
    else
      CP.fCost=g_score[map->getCellRowID(neighborCell)][map->getCellColID(neighborCell)]+ calculateHCost(map,neighborCell,goalCell);      
 
    OPL.push_back(CP);

}
/*******************************************************************************/
//Function Name: isContains
//Inputs: the list, the cellID
//Output: true or false
//Description: it is used to check if a cell exists in the open list
/*********************************************************************************/
bool RAStar::isContains(list<cells> & list1, int cellID)
{
  for (list<cells>::iterator it = list1.begin(); it != list1.end(); it++){
    if (it->currentCell == cellID) 
	return true;
  }  
  return false; 
}
/*******************************************************************************/
//Function Name: calculateHCost
//Inputs:the cellID and the goalCell
//Output: the distance between the current cell and the goal cell
//Description: it is used to calculate the hCost 
/*********************************************************************************/
float RAStar::calculateHCost(OccupancyGridMap* map,int cellID, int goalCell)
{    
  int x1=map->getCellRowID(goalCell);
  int y1=map->getCellColID(goalCell);
  int x2=map->getCellRowID(cellID);
  int y2=map->getCellColID(cellID);
  
  if(getNeighborNumber()==4) 
    //The diagonal shortcut distance between two grid points (x1,y1) and (x2,y2) is:
      return min(abs(x1-x2),abs(y1-y2))*sqrt(2) + max(abs(x1-x2),abs(y1-y2))-min(abs(x1-x2),abs(y1-y2));
  
  else
    //manhatten distance for 8 neighbor
    return abs(x1-x2)+abs(y1-y2);
}
/*******************************************************************************/
//Function Name: searchCellInList
//Inputs:the cellID, the list
//Output: index of the cell in the list
//Description: it is used to search the index of a cell in a list
/*********************************************************************************/
list<cells>::iterator RAStar::getPositionInList(list<cells> & list1, int cellID)
{
  for (list<cells>::iterator it = list1.begin(); it != list1.end(); it++){
    if (it->currentCell == cellID) 
	return it;
  }  
}
/*******************************************************************************/
//Function Name: compareFCost
//Inputs: 
//Output:
//Description: comparaison function
/*********************************************************************************/
bool RAStar::_compareFCost(cells const &c1, cells const &c2) 
{ 
  return c1.fCost < c2.fCost; 
}
/*******************************************************************************/
//Function Name: constructPath
//Inputs: the start and the goal Cells
//Output: the best path
//Description: it is used to construct the robot path
/*********************************************************************************/
Path* RAStar::constructPath(OccupancyGridMap* map,float** g_score, int startCell, int goalCell)
{
  Path* bestPath=new Path();
  Path* path=new Path();
 
  
  path->insertCell(map,bestPath->getPath().size(),goalCell);
  int currentCell=goalCell;
  
  while(currentCell!=startCell)
  { 
     vector <int> neighborCells;
     // search the neighbor that has the minimum g_score
     if (getNeighborNumber()==8)
        neighborCells=findFreeNeighborCell(map, currentCell);
     else
     {
       if (getNeighborNumber()==4)
	  neighborCells=findfourFreeNeighborCell(map, currentCell);
     }
       
     vector <float> gScoresNeighbors;
     for(uint i=0; i<neighborCells.size(); i++)
     {
       gScoresNeighbors.push_back(g_score[map->getCellRowID(neighborCells[i])][map->getCellColID(neighborCells[i])]);
     }
     int posMinGScore=distance(gScoresNeighbors.begin(), min_element(gScoresNeighbors.begin(), gScoresNeighbors.end()));
     currentCell=neighborCells[posMinGScore];
     
     //insert the neighbor in the path
     path->insertCell(map,path->getPath().size(),currentCell);
  }
  for(uint i=0; i<path->getPath().size(); i++)
     bestPath->insertCell(map,bestPath->getPath().size(),path->getPath()[path->getPath().size()-(i+1)]);
  
  return bestPath;
}
/*******************************************************************************/
//Function Name: findPath
//Inputs: the map layout, the start and the goal Cells and a boolean to indicate if we will use break ties or not
//Output: the best path
//Description: it is used to generate the robot free path
/*********************************************************************************/
Path* RAStar::findPath(OccupancyGridMap* map, int startCell, int goalCell)
{
  Path* bestPath=new Path();
   Path* emptyPath=new Path();
  cells CP;
  list<cells> OPL=getOpenList();
  float tBreak = 1+1/(map->getHeight()+map->getWidth());  // coefficient for breaking ties

  
  g_score = (float**) malloc(sizeof( float*)*map->getHeight());
  for(int i=0;i<map->getWidth();i++)
  g_score[i] = (float*) malloc(sizeof(float)*map->getWidth());
  
  int currentCell;
  
  //for each vertex in the grid g_score=infinity
  for (uint i=0; i< map->getHeight(); i++)
    for(uint j=0; j< map->getWidth(); j++)
    {
      g_score[i][j]=std::numeric_limits< float >::infinity();
    }
  
  //calculate g_score and f_score of the start position
  g_score[map->getCellRowID(startCell)][map->getCellColID(startCell)]=0;
   //add the start cell to the open list
  CP.currentCell=startCell;
  if (getIsWithBreakTies()==true)
    CP.fCost=g_score[map->getCellRowID(startCell)][map->getCellColID(startCell)]+tBreak*calculateHCost(map,startCell,goalCell);
  else
    CP.fCost=g_score[map->getCellRowID(startCell)][map->getCellColID(startCell)]+calculateHCost(map,startCell,goalCell);
  
  OPL.push_back(CP);
  
  currentCell=startCell;
  
  
  //while the open list is not empty continuie the search or g_score(goalCell) is equal to infinity
while (!OPL.empty()&& g_score[map->getCellRowID(goalCell)][map->getCellColID(goalCell)]==numeric_limits< float >::infinity()) 
{
    //choose the cell that has the lowest cost fCost in the open list
    cells COfCells=*min_element(OPL.begin(), OPL.end(), &RAStar::_compareFCost);
    currentCell=COfCells.currentCell;
    
    //remove the currentCell from the openList
     list<cells>::iterator it=getPositionInList(OPL,currentCell); 
     list<cells>::iterator it1=OPL.erase(it);
   
    //search the neighbors of the current Cell
    vector <int> neighborCells; 
    if(getNeighborNumber()==8)
       neighborCells=findFreeNeighborCell(map, currentCell);
    else
      if(getNeighborNumber()==4)
	 neighborCells=findfourFreeNeighborCell(map, currentCell);
   
    for(uint i=0; i<neighborCells.size(); i++) //for each neighbor v of current cell
    {
	  // if the g_score of the neighbor is equal to INF: unvisited cell
	  if(g_score[map->getCellRowID(neighborCells[i])][map->getCellColID(neighborCells[i])]==numeric_limits< float >::infinity())
	  {
	      g_score[map->getCellRowID(neighborCells[i])][map->getCellColID(neighborCells[i])]=g_score[map->getCellRowID(currentCell)][map->getCellColID(currentCell)]
	                                                                                      +map->getMoveCost(currentCell,neighborCells[i]);
	
    
             //verify if the neighbor does not exist in the open List and add it
	     
	      if(!isContains(OPL,neighborCells[i]))
	           addNeighborCellToOpenList(map, OPL, neighborCells[i],g_score,goalCell,tBreak);    
	}//end if
    }//end for
}//end while
  
  if(g_score[map->getCellRowID(goalCell)][map->getCellColID(goalCell)]!=numeric_limits< float >::infinity())  // if g_score(goalcell)==INF : construct path 
  {
      bestPath=constructPath(map,g_score,startCell,goalCell);
      return   bestPath;
    
  }
  else
  {
    cout << "Failure to find a path !" << endl;
    return emptyPath;
  }

}

