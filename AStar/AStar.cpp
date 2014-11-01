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

#include "AStar.h"
#include<iostream>
#include<fstream>
#include <stdio.h>
#include <algorithm> 
#include <list>
#include <math.h>

using namespace std;

//ofstream file2 ("AS.txt", ios::trunc) ;
/**********************************************************/
//Function: Constructor
/**********************************************************/

AStar::AStar(bool BT)
{
  list<coupleOfCells> OPL;
  list<coupleOfCells> CSL;
  
  setOpenList(OPL);
  setClosedList(CSL);
  setIsWithBreakTies(BT);
}
/**********************************************************/
//Function: destructor
/**********************************************************/
AStar::~AStar(void){
}
/**********************************************************/
//Function: Mutators
/**********************************************************/
void AStar::setOpenList(list<coupleOfCells> OPL){
  openList=OPL;
}
void AStar::setClosedList(list<coupleOfCells> CSL){
   closedList=CSL;
}
void AStar::setIsWithBreakTies(bool withBT)
{
  isWithBreakTies=withBT;
}
/**********************************************************/
//Function: Accessors
/**********************************************************/
list <coupleOfCells> AStar::getOpenList(){
   return openList;
 }
list <coupleOfCells> AStar::getClosedList(){
   return closedList;
}
bool AStar::getIsWithBreakTies()
{
  return isWithBreakTies;
}
/*******************************************************************************/
//Function Name: addNeighborCellsToOpenList
//Inputs: the open list, the neighbors Cells and the parent Cell
//Output: 
//Description: it is used to add the neighbor Cells to the open list
/*********************************************************************************/
void AStar::addNeighborCellsToOpenList(OccupancyGridMap* map, list<coupleOfCells> & OPL, vector <int> neighborCells, int parent, float gCostParent, int goalCell,float tBreak)
{
  vector <coupleOfCells> neighborsCellsOrdered;
  for(uint i=0; i< neighborCells.size(); i++)
  {
    coupleOfCells CP;
    CP.currentCell=neighborCells[i]; //insert the neighbor cell
    CP.parentCell=parent; //insert the parent cell
    //calculate the gCost
    CP.gCost=gCostParent+map->getMoveCost(parent,neighborCells[i]);
      
    //calculate the hCost: Euclidian distance from the neighbor cell to the goalCell
    CP.hCost=calculateHCost(map,neighborCells[i],goalCell);
    //calculate fcost
    CP.fCost=CP.gCost+tBreak*CP.hCost;
   // neighborsCellsOrdered.push_back(CP);
    OPL.push_back(CP);
  }
}
/*******************************************************************************/
//Function Name: isContains
//Inputs: the list, the cellID
//Output: true or false
//Description: it is used to check if a cell exists in the open list or in the closed list
/*********************************************************************************/
bool AStar::isContains(list<coupleOfCells> & list1, int cellID)
{
  for (list<coupleOfCells>::iterator it = list1.begin(); it != list1.end(); it++){
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
float AStar::calculateHCost(OccupancyGridMap* map,int cellID, int goalCell)
{
  return sqrt((pow((map->getCellRowID(cellID)-map->getCellRowID(goalCell)),2))+pow((map->getCellColID(cellID)-map->getCellColID(goalCell)), 2));
}
/*******************************************************************************/
//Function Name: recalculateGCost
//Inputs:the mapLayout, the vector of neighbor cells that exist in the open list 
//Description: it is used to calculate the new gCost of the neighbor cells that exist in the open list and compare them to the old gCost
/*********************************************************************************/
void AStar::recalculateGCost(OccupancyGridMap* map, list<coupleOfCells> & OPL, vector<int> neighborCellsInOpenList,int currentCell, float gCostCurrent)
{
  
  for(uint i=0; i< neighborCellsInOpenList.size(); i++)
  {
    coupleOfCells CP;
     
    //compare the gCost that exist in the openlist with the new gcost
    list<coupleOfCells>::iterator it=getPositionInList(OPL,neighborCellsInOpenList[i]);
    float previousGCost=(*it).gCost;
  
    //if the new gCost is better than the cost that exist in OPL: update the gCost
    if(previousGCost > gCostCurrent+map->getMoveCost(currentCell,neighborCellsInOpenList[i]))
    {
      CP.currentCell=(*it).currentCell;
      CP.parentCell=currentCell;
      CP.gCost=gCostCurrent+map->getMoveCost(currentCell,neighborCellsInOpenList[i]);
      CP.hCost=(*it).hCost;
      CP.fCost=CP.gCost+CP.hCost;
      it=OPL.erase(it);
      OPL.insert (it,CP);
    }
  }
}
/*******************************************************************************/
//Function Name: getPositionInList
//Inputs:the cellID, the list
//Output: index of the cell in the list
//Description: it is used to search the index of a cell in a list
/*********************************************************************************/
list<coupleOfCells>::iterator AStar::getPositionInList(list<coupleOfCells> & list1, int cellID)
{
  for (list<coupleOfCells>::iterator it = list1.begin(); it != list1.end(); it++){
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
bool AStar::_compareFCost(coupleOfCells const &c1, coupleOfCells const &c2) 
{ 
  return c1.fCost < c2.fCost; 
}
/*******************************************************************************/
//Function Name: constructPath
//Inputs: the start and the goal Cells
//Output: the best path
//Description: it is used to construct the robot path
/*********************************************************************************/
Path* AStar::constructPath(OccupancyGridMap* map,list<coupleOfCells> & CSL, int startCell, int goalCell)
{
  Path* bestPath=new Path();
  Path* path=new Path();
  path->insertCell(map,bestPath->getPath().size(),goalCell);
  int parentCell=goalCell;
  
  while(parentCell!=startCell)
  {
     list<coupleOfCells>::iterator it=getPositionInList(CSL,parentCell);
     parentCell=(*it).parentCell;
     path->insertCell(map,path->getPath().size(),parentCell);
  }
  for(uint i=0; i<path->getPath().size(); i++)
     bestPath->insertCell(map,bestPath->getPath().size(),path->getPath()[path->getPath().size()-(i+1)]);
  
  return bestPath;
}
/*******************************************************************************/
//Function Name: findPath
//Inputs: the map layout, the start and the goal Cells
//Output: the best path
//Description: it is used to generate the robot free path
/*********************************************************************************/
Path* AStar::findPath(OccupancyGridMap* map, int startCell, int goalCell)
{
  
  Path* bestPath=new Path();
  Path* emptyPath=new Path();
  coupleOfCells CP;
  list<coupleOfCells> OPL=getOpenList();
  list<coupleOfCells> CSL=getClosedList();
  int currentCell;
  float tBreak = 1+1/(map->getHeight()+map->getWidth());  // coefficient for breaking ties
  
  //add the start cell to the open list
  CP.currentCell=startCell;
  CP.parentCell=startCell;
  CP.gCost=0;
  CP.hCost=calculateHCost(map,startCell,goalCell);
  
   if (getIsWithBreakTies()==true)
    CP.fCost=CP.gCost+tBreak*CP.hCost;
   else
    CP.fCost=CP.gCost+CP.hCost;
  
  OPL.push_back(CP);

  currentCell=startCell;
 
  while (!OPL.empty()) //while the open list is not empty continuie the search
  {
    
    //choose the cell that has the lowest cost fCost in the open list
    coupleOfCells COfCells=*min_element(OPL.begin(), OPL.end(), &AStar::_compareFCost);
    currentCell=COfCells.currentCell;
    
    // put the currentCell in the closedList
    list<coupleOfCells>::iterator it=getPositionInList(OPL,currentCell);    
    CP.currentCell=currentCell;
    CP.parentCell=(*it).parentCell;
    CP.gCost=(*it).gCost;
    CP.hCost=(*it).hCost;
    CP.fCost=(*it).fCost;
    CSL.push_back(CP);
    
    // if the currentCell is the goalCell: success: path found
    if(currentCell==goalCell)
    {     
      bestPath=constructPath(map, CSL, startCell, goalCell);  
      return bestPath;
    }
        
    //remove the currentCell from the openList
     list<coupleOfCells>::iterator it1=OPL.erase(it);
    
    //search the neighbors of the current Cell
    vector <int> neighborCells=findFreeNeighborCell(map, currentCell);
     
    //neighbors that exist in the closedList are ignored 
    vector <int> neighborNotInClosedList;
    for(uint i=0; i<neighborCells.size(); i++)
    {
      if(!isContains(CSL,neighborCells[i]))
      {
	neighborNotInClosedList.push_back(neighborCells[i]);
      }
    }
    
    //search the neighbors that already exist in the open List
    vector <int> neighborsInOpenList;
    vector <int> neighborsNotInOpenList;
    for(uint i=0; i<neighborNotInClosedList.size(); i++)
    {
      if(isContains(OPL,neighborNotInClosedList[i]))
	neighborsInOpenList.push_back(neighborNotInClosedList[i]);
      else
	neighborsNotInOpenList.push_back(neighborNotInClosedList[i]);
    }
    
    
    //add the neighbors that are not in the open list to the open list and mark the current cell as their parent

     addNeighborCellsToOpenList(map, OPL, neighborsNotInOpenList, currentCell, CP.gCost, goalCell,tBreak);
  
    //For the neighbors that are already in the open list: check their costs gCost  and update them if necessary
    recalculateGCost(map, OPL, neighborsInOpenList, currentCell, CP.gCost);
     
  }
  if(OPL.empty())  // if the openList is empty: then failure to find a path   
  {
      cout << "Failure to find a path !" << endl;
      return emptyPath;
     // exit(1);
  }

}

