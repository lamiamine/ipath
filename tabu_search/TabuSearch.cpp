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


#include "TabuSearch.h"

#include<iostream>
#include<fstream>
#include <stdio.h>
#include <algorithm> 
#include <list>
#include <sstream>
using namespace std;

//ofstream file ("TS.txt", ios::trunc) ;


/**********************************************************/
//Function: Constructor
/**********************************************************/
TabuSearch::TabuSearch()
{
}
TabuSearch::TabuSearch(int size,int t,int numberOfIterations){  
  Path* path;
  list<TabuMove> TLIN;
  //TLIN.resize(size);
  list<TabuMove> TLOUT;
 // TLOUT.resize(size); 
  setNumberOfIterations(numberOfIterations);
  setTabuListSize(size);
  /*********************/
 
  setTabuListOut(TLOUT); 
  /*******************/
  
  setTabuListIn(TLIN);
  /**********************************/
  setTenure(t);
  setBestPath(path);
   
}
/**********************************************************/
//Function: destructor
/**********************************************************/
TabuSearch::~TabuSearch(void){
}
/**********************************************************/
//Function: Mutators
/**********************************************************/
void TabuSearch::setBestPath(Path* path){
  bestPath=path;
}
void TabuSearch::setTabuListIn(list<TabuMove> TLIN){
  tabuListIn=TLIN;
}
void TabuSearch::setTabuListOut(list<TabuMove> TLOUT){
   tabuListOut=TLOUT;
}
void TabuSearch::setTabuListSize(int size){
  tabuListSize=size;
}
void TabuSearch::setTenure(int t){
  tenure=t;
}
/**********************************************************/
//Function: Accessors
/**********************************************************/
Path* TabuSearch::getBestPath(){
  return bestPath;
}
list<TabuMove> TabuSearch::getTabuListIn(){
  return tabuListIn;
}
list<TabuMove> TabuSearch::getTabuListOut(){
  return tabuListOut;
}
int TabuSearch::getTabuListSize(){
  return tabuListSize;
}
int TabuSearch::getTenure(){
  return tenure;
}
/*******************************************************************************/
//Function Name: isTabu
//Inputs: the two cell IDs and the tabuList
//Output: a boolean variable
//Description: it is used to verify if a move is tabu or not
/*********************************************************************************/
bool TabuSearch::isTabu(int cellID1, int cellID2,list<TabuMove>  TabuList)
{ 
  for (list<TabuMove>::iterator it = TabuList.begin(); it != TabuList.end(); it++){
    if((it->getFromCell() == cellID1) && (it->getToCell() == cellID2))
      if (it->getTenure1() != 0)
	return true;
      else
	return false; 
  }
}
/*******************************************************************************/
//Function Name: addTabuMove
//Inputs: the two cell IDs and the tabuList
//Output: 
//Description: it is used to make the move cellID1,cellID2 tabu
/*********************************************************************************/
void TabuSearch::addTabuMove(int cellID1, int cellID2, list<TabuMove> & TabuList, int iteration, bool aspirationCriteria)
{
  if(!aspirationCriteria)
  {
      TabuMove addMove;
      addMove.setFromCell(cellID1);
      addMove.setToCell(cellID2);
      addMove.setTenure1(getTenure()); 
      addMove.setDateOfExpiration(getTenure()+iteration);
      TabuList.push_back(addMove);
  }
  else
    changeTenureValue(cellID1, cellID2, TabuList, iteration);
}
/*******************************************************************************/
//Function Name: changeTenureValue
//Inputs: the two cell IDs and the tabuList
//Output: 
//Description: it is used to change the tenure value of a move that already exists in the tabu list
/*********************************************************************************/
void TabuSearch::changeTenureValue(int cellID1, int cellID2, list<TabuMove> & TabuList, int iteration)
{
   for(list<TabuMove>::iterator it = TabuList.begin(); it !=TabuList.end(); it++)
    {
      if((((*it).getFromCell()==cellID1) && ((*it).getToCell()==cellID2)) || (((*it).getFromCell()==cellID2) && ((*it).getToCell()==cellID2)))
      {
	(*it).setDateOfExpiration(getTenure()+iteration);
      }
    }
}
/*******************************************************************************/
//Function Name: swapMove
//Inputs: the path, the index of the cell which will be swapped and the new cell
//Output: the new path after exchanging the two cells if the move is not tabu
//Description: it is used to exchange two cells in the current path if this move is not tabu
/*********************************************************************************/
Path* TabuSearch::swapMove(OccupancyGridMap* map,Path* path,int IndexofCellToBeReplaced, int NewCellID, bool & aspirationCriteria)
{
  
  //the new path after the swap move.
  Path* newPath=new Path();
  newPath->setName("The new Path in Swap ");
  // check if the move that consist in adding a new arc between the newCell and the path-neighbors of cell to be replaced
  bool isTabuIn1 = isTabu(path->getPath()[IndexofCellToBeReplaced-1],NewCellID,getTabuListIn());
  bool isTabuIn2 = isTabu(NewCellID,path->getPath()[IndexofCellToBeReplaced+1],getTabuListIn());
 // check if the move that consist in remove an arc cell to be replaced and its path-neighbors
  bool isTabuOut1 = isTabu(path->getPath()[IndexofCellToBeReplaced-1],path->getPath()[IndexofCellToBeReplaced],getTabuListOut());
  bool isTabuOut2 = isTabu(path->getPath()[IndexofCellToBeReplaced],path->getPath()[IndexofCellToBeReplaced+1],getTabuListOut());
  //perform the swap
    newPath->setPath(path->getPath());
    newPath->setCost(path->getPathCost(map,false));
    newPath->setCell(map,NewCellID, IndexofCellToBeReplaced);
  //if the move is not tabu
  if(!isTabuIn1 && !isTabuIn2 && !isTabuOut1 && !isTabuOut2)
  {
      return newPath;
  }
   //if the move is tabu then check if it verify the aspiration criteria else return the current path
   else
   {
     //check if the move improves the current path cost: aspiration criteria
     if(newPath->getPathCost(map,false) <= path->getPathCost(map,false))
     {
          aspirationCriteria=true;
	  return newPath;
     }
     
     else
      return path;
   }
}

/*******************************************************************************/
//Function Name: insertMove
//Inputs: the path, the index of the cell which will be insert and the new cell
//Output: the new path after insertion of the new cell if the move is not tabu
//Description: it is used to insert a new cell in the current path if this move is not tabu
/*********************************************************************************/
Path* TabuSearch::insertMove(OccupancyGridMap* map,Path* path,int IndexofCellToBeInserted, int NewCellID, bool & aspirationCriteria)
{
  Path* newPath=new Path();//the new path after the insert move. 
  newPath->setName("The new Path in Insert ");
  // check if the move is tabu or not
  bool isTabuIn1 = isTabu(path->getPath()[IndexofCellToBeInserted-1],NewCellID,getTabuListIn());
  bool isTabuIn2 = isTabu(NewCellID,path->getPath()[IndexofCellToBeInserted],getTabuListIn());
  bool isTabuOut3 = isTabu(path->getPath()[IndexofCellToBeInserted-1],path->getPath()[IndexofCellToBeInserted],getTabuListOut());
  newPath->setPath(path->getPath());
  newPath->setCost(path->getPathCost(map,false));
  newPath->insertCell (map,IndexofCellToBeInserted,NewCellID);
  //if the move is not tabu
  if(!isTabuIn1 && !isTabuIn2 && !isTabuOut3) 
  { 
    return newPath;
  }
   //if the move is tabu then check if it verify the aspiration criteria else return the current path
   else
   {
     //check if the move improves the current path cost: aspiration criteria
     if(newPath->getPathCost(map,false) <= path->getPathCost(map,false))
     {
          aspirationCriteria=true;
	  return newPath;
     }
     
     else
      return path;
   }
}
/*******************************************************************************/
//Function Name: removeMove
//Inputs: the path, the index of the cell which will be removed 
//Output: the new path after removing the cell if the move is not tabu
//Description: it is used to remove a cell from the current path if this move is not tabu
/*********************************************************************************/
Path* TabuSearch::removeMove(OccupancyGridMap* map,Path* path,int IndexofCellToBeRemoved,bool & aspirationCriteria)
{
  Path* newPath=new Path();//the new path after the insert move. 
  newPath->setName("The new Path in Remove ");
  bool isTabuIn1;
  bool isTabuOut1;
  bool isTabuOut2;
    isTabuIn1=isTabu(path->getPath()[IndexofCellToBeRemoved-1],path->getPath()[IndexofCellToBeRemoved+1],getTabuListIn());
    isTabuOut1=isTabu(path->getPath()[IndexofCellToBeRemoved-1],path->getPath()[IndexofCellToBeRemoved],getTabuListOut()); 
    isTabuOut2=isTabu(path->getPath()[IndexofCellToBeRemoved],path->getPath()[IndexofCellToBeRemoved+1],getTabuListOut());
    
    //perform the remove
      newPath->setPath(path->getPath());
      newPath->setCost(path->getPathCost(map,false));
      newPath->removeCell(map,IndexofCellToBeRemoved);   
      
    //if the move is not tabu
    if(!isTabuOut1 && !isTabuOut2 && !isTabuIn1)
    { 
      return newPath;
    }
  else 
  {
      //check if the move improves the current path cost: aspiration criteria
     if(newPath->getPathCost(map,false) <= path->getPathCost(map,false))
     {
          aspirationCriteria=true;
	  return newPath;
     }
     
     else
      return path;
  }
}

/*******************************************************************************/
//Function Name: findUnvisitedCommonNeighbors
//Inputs: the 2 cells
//Output: a vector that contains the neighbor cells
//Description: it generates the unvisited common neighbors of two cells
/*********************************************************************************/
vector <int> TabuSearch::findUnvisitedCommonNeighbors(OccupancyGridMap* map,Path* path,  int cellID1, int cellID2)
{
  vector <int> unvisitedCommonNeighbors;
  vector <int> neighborsCell1;
  vector <int> neighborsCell2;
  
  neighborsCell1=findFreeNeighborCell (map,cellID1);
  neighborsCell2=findFreeNeighborCell (map,cellID2);
  
  for (uint i=0; i<neighborsCell1.size(); i++)
  {
    for(uint j=0; j<neighborsCell2.size(); j++)
    {
      if(neighborsCell2[j]==neighborsCell1[i] && !contains(neighborsCell2[j], path->getPath())) 
      {
	unvisitedCommonNeighbors.push_back(neighborsCell2[j]);
	break;
      }
      
    }
  } 
  return unvisitedCommonNeighbors;
}
/*******************************************************************************/
//Function Name: findUnvisitedCommonNeighborsInBloc
//Inputs: the 2 cells
//Output: a vector that contains the neighbor cells
//Description: it generates the unvisited common neighbors of two cells
/*********************************************************************************/
vector <int> TabuSearch::findUnvisitedCommonNeighborsInBloc(OccupancyGridMap* map,Path* currentBloc, vector<Path*> previousBestBlocs,  int cellID1, int cellID2)
{
  vector <int> unvisitedCommonNeighbors;
  
  vector <int> unvisitedCommonNeighbors1=findUnvisitedCommonNeighbors(map,currentBloc,cellID1, cellID2); //find the unvisited common neighbors of two cells
  for(uint i=0; i<unvisitedCommonNeighbors1.size(); i++)
  {
    int j=0; bool verify=false;
    while (j<previousBestBlocs.size() && verify==false)
    {
      if(contains(unvisitedCommonNeighbors1[i], previousBestBlocs[j]->getPath()))
       verify=true;
      j++;
    }
    if(!verify)
     unvisitedCommonNeighbors.push_back(unvisitedCommonNeighbors1[i]); 
  } 
  return unvisitedCommonNeighbors;
}
/*******************************************************************************/
//Function Name: findBestPath
//Inputs: a vector of paths
//Output: index of the best path in the vector
//Description: it is to generate the best path from a set of candidate paths 
/*********************************************************************************/
int TabuSearch::findBestPath(OccupancyGridMap* map, vector<Path*> paths)
{
  int indexBestPath=0;
  float minPathCost=paths[0]->getPathCost(map,false);
  
  for (uint i=0; i<paths.size(); i++)
  {
    if(paths[i]->getPathCost(map,false)< minPathCost)
    {
      minPathCost=paths[i]->getPathCost(map,false);
      indexBestPath=i;
    }
  }

  return indexBestPath;
}
/*******************************************************************************/
//Function Name: findBestNeighbor
//Inputs: a vector of neighbors
//Output: index of the best neighbor in the vector
//Description: it is to generate the best neighbor from a set of neighbors 
/*********************************************************************************/
int TabuSearch::findBestNeighbor(OccupancyGridMap* map, vector<Move> neighborhood)
{
  int indexBestPath=0;
  float minPathCost=neighborhood[0].newPathAfterMove->getPathCost(map,false);
  
  for (uint i=0; i<neighborhood.size(); i++)
  {
    if(neighborhood[i].newPathAfterMove->getPathCost(map,false)< minPathCost)
    {
      minPathCost=neighborhood[i].newPathAfterMove->getPathCost(map,false);
      indexBestPath=i;
    }
  }
 
  return indexBestPath;
}
/*******************************************************************************/
//Function Name: diff
//Inputs: 
//Output: 
//Description: it is used to calculate the execution time
/*********************************************************************************/
timespec TabuSearch::diff(timespec start, timespec end)
{
  timespec temp;
	if ((end.tv_nsec-start.tv_nsec)<0) {
		temp.tv_sec = end.tv_sec-start.tv_sec-1;
		temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
	} else {
		temp.tv_sec = end.tv_sec-start.tv_sec;
		temp.tv_nsec = end.tv_nsec-start.tv_nsec;
	}
	return temp;
}
int clock_gettime(clockid_t clk_id, struct timespect *tp);
/*******************************************************************************/
//Function Name: findPath
//Inputs: 
//Output: 
//Description: it is used to generate N paths after N iterations 
/*********************************************************************************/
/*Path* TabuSearch::findPath(OccupancyGridMap* map, Path* initialPath)
{
   ofstream MyExcelFile1;
   string statisticFileName = "exectionTimes.ods";
   MyExcelFile1.open(statisticFileName.c_str(), ios::app);  
  
  clock_t startTime;
  timespec time1, time2;
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);
  vector <double> bestExecutionTimes;
  vector <Path*> bestPaths; 
  Path* currentPath=new Path();
  int NITS;
  vector<int> unvisitedCommonNeighbors;

  
  vector <Move> neighborhood;
  
  currentPath->setPath(initialPath->getPath());
  currentPath->setCost(initialPath->getPathCost(map,false));
  list<TabuMove>TLIN=getTabuListIn();
 
  list<TabuMove>TLOUT=getTabuListOut();

 // cout << "hereee" << endl;
  NITS=0;
  startTime=getCurrentTime1();
  while (NITS < getNumberOfIterations()) //repeat the Tabu Search algorithm NumberOfIterations times
  {
    neighborhood.clear();
    for (uint i=1; i<currentPath->getPath().size()-1; i++) //for each cell (except the start and the goal cells)
    {
      //Make the swap Move
     unvisitedCommonNeighbors=findUnvisitedCommonNeighbors(map,currentPath,currentPath->getPath()[i-1],currentPath->getPath()[i+1]);     
     for(uint j=0; j<unvisitedCommonNeighbors.size();j++)
      { 
	if (unvisitedCommonNeighbors[j]!=currentPath->getPath()[i])
	{
	  bool aspirationCriteria=false;
	  Path* newPathAfterSwap=swapMove(map,currentPath,i, unvisitedCommonNeighbors[j],aspirationCriteria);
	  if ((newPathAfterSwap->getPathCost(map,false)!= currentPath->getPathCost(map,false)) || ( (newPathAfterSwap->getPathCost(map,false) == currentPath->getPathCost(map,false)) && newPathAfterSwap!=currentPath))
	  {
	  Move swap;
	    swap.newPathAfterMove=newPathAfterSwap;
	    swap.previousCell=currentPath->getPath()[i];
	    swap.position=i;
	    swap.moveType="Swap";
	    swap.aspirationCriteria=aspirationCriteria;
	    neighborhood.push_back(swap);
	   // currentPath = newPathAfterSwap;
	  }
	}
       }
       
      //Make Insert Move 
      unvisitedCommonNeighbors.clear();
      unvisitedCommonNeighbors=findUnvisitedCommonNeighbors(map,currentPath,currentPath->getPath()[i-1],currentPath->getPath()[i]); 
      for(uint j=0; j<unvisitedCommonNeighbors.size();j++)
      {
	bool aspirationCriteria=false;
	Path* newPathAfterInsert=insertMove(map,currentPath,i, unvisitedCommonNeighbors[j],aspirationCriteria);
	if ((newPathAfterInsert->getPathCost(map,false)!= currentPath->getPathCost(map,false)) || ( (newPathAfterInsert->getPathCost(map,false) == currentPath->getPathCost(map,false)) && newPathAfterInsert!=currentPath))
	{
	Move insert;
	  insert.newPathAfterMove=newPathAfterInsert;
	  insert.previousCell=currentPath->getPath()[i];
	  insert.position=i;
	  insert.moveType="Insert";
	  insert.aspirationCriteria=aspirationCriteria;
	 neighborhood.push_back(insert);
	}
      }
  
     //Make Remove Move
    vector<int> neighborCells=findFreeNeighborCell (map,currentPath->getPath()[i-1]); 
     if(contains(currentPath->getPath()[i+1], neighborCells))
     {
       
       bool aspirationCriteria=false;
       Path* newPathAfterRemove=removeMove(map,currentPath,i,aspirationCriteria);
       if ((newPathAfterRemove->getPathCost(map,false)!= currentPath->getPathCost(map,false)) || ( (newPathAfterRemove->getPathCost(map,false) == currentPath->getPathCost(map,false)) && newPathAfterRemove!=currentPath))
       {
        Move remove;
	  remove.newPathAfterMove=newPathAfterRemove;
	  remove.previousCell=currentPath->getPath()[i];
	  remove.position=i;
	  remove.moveType="Remove";
	  remove.aspirationCriteria=aspirationCriteria;
	  neighborhood.push_back(remove);
          currentPath = newPathAfterRemove;
       }
     }
    }
  
    //search the best path and put it in the vector bestPaths

   if(neighborhood.size() != 0 )
   { 
      //Search the best neighbor
      int indexBestPathAfterOneIteration=findBestNeighbor(map,neighborhood);
     
      //make the move tabu
      makeTheMoveTabu (neighborhood, indexBestPathAfterOneIteration,TLIN,TLOUT, NITS);
      //put the best path in the vector bestPaths
      bestPaths.push_back(neighborhood[indexBestPathAfterOneIteration].newPathAfterMove);
      currentPath=neighborhood[indexBestPathAfterOneIteration].newPathAfterMove;
      clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
      cout<<"time to generate best path= " << (diff(time1,time2).tv_nsec)*1e-6<< "nanoseconde" << endl;
      double BestPathTime=getDuration1( startTime);
      //bestExecutionTimes.push_back(BestPathTime);
      bestExecutionTimes.push_back((diff(time1,time2).tv_nsec)*1e-6);
      cout<<"Current Best Path Cost: "<<neighborhood[indexBestPathAfterOneIteration].newPathAfterMove->getCost() <<"at iteration "<<NITS<< " time= " <<BestPathTime <<endl;
    }
   
    // After an iteration:update the tabu list; 
    updateTabuLists(TLIN, TLOUT, NITS);
    
    setTabuListIn(TLIN);
    setTabuListOut(TLOUT);
   /*file << "After " << endl;     
	    file << "iteration N° = " << NITS << endl;
	    file << "******TabuListIN*********" << endl;
	    file << ".....sizeTLIN.... " << TLIN.size() << endl;
	    if (TLIN.size()!=0) 
	     {  
	       for (list<TabuMove>::iterator it = TLIN.begin(); it != TLIN.end(); it++){
	 	    
	 	  file<<"FromCell= " << (*it).getFromCell() << " , ToCell= " << (*it).getToCell() << " , Tenure= " << (*it).getTenure1()  << ", Date" << (*it).getDateOfExpiration() <<endl;
	       }
	       	
	     }
	     file << "******TabuListOUT*********" << endl;
	     file << ".....sizeTLOUT.... " << TLOUT.size() << endl;
	     if (TLOUT.size()!=0) 
	     {  
	       for (list<TabuMove>::iterator it = TLOUT.begin(); it != TLOUT.end(); it++){
	 	    
	 	  file<<"FromCell= " << (*it).getFromCell() << " , ToCell= " << (*it).getToCell() << " , Tenure= " << (*it).getTenure1() <<  ", Date" << (*it).getDateOfExpiration() <<endl;
		 
	      }
	    file << "-----------------------------------------------------------------" << endl; 	
	    }*/
    
  
  /* NITS++;
  }
  //search of the best path after NITS iterations
  if(bestPaths.size()!=0)
  {  
     int indexBestPath=findBestPath(map,bestPaths);
    
     MyExcelFile1 <<initialPath->getPathCost(map,false) << " " << bestExecutionTimes[indexBestPath]<<endl;
     return bestPaths[indexBestPath];
  }
  else
    return initialPath;
}*/
/*******************************************************************************/
//Function Name: findPath
//Inputs: 
//Output: 
//Description: it is used to generate N paths after N iterations 
/*********************************************************************************/
Path* TabuSearch::findPath(OccupancyGridMap* map, Path* initialPath, timespec & bestExecutionTime)
{ 
 
  vector <timespec> bestExecutionTimes;
  int sizeBloc;
  if (initialPath->getPath().size()>=5)
   sizeBloc=5; //The size of one bloc
  else
   sizeBloc=initialPath->getPath().size(); 
  vector <Path*> bestPaths; 
  vector <Path*> bestBlocs;
  Path* currentPath=new Path();
  int NITS;
  vector<int> unvisitedCommonNeighbors;

  
  vector <Move> neighborhoodBloc;
  
  currentPath->setPath(initialPath->getPath());
  currentPath->setCost(initialPath->getPathCost(map,false));
  list<TabuMove>TLIN=getTabuListIn();
  list<TabuMove>TLOUT=getTabuListOut();

  
  
  NITS=0;
  timespec time1,time2;
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);

  while (NITS < getNumberOfIterations()) //repeat the Tabu Search algorithm NumberOfIterations times
  {
   
    //The number of the bloc in a path
    int numberOfBlocs=currentPath->getPath().size()/sizeBloc; //The number of blocs in a path
    int sizeOfTheLastBloc=sizeBloc+ (currentPath->getPath().size()%sizeBloc); // The size of the last bloc: it may contains more cells than the other blocs
    int j=0;
    int g=0;
    Path* currentBloc;// The current bloc
    bestBlocs.clear(); // The best blocs after one iteratio
    for(uint i=0; i< numberOfBlocs; i++)
    {
      //devise the current path to blocs
      neighborhoodBloc.clear();
      currentBloc=new Path(); 
      if(i==numberOfBlocs-1 && sizeOfTheLastBloc!=sizeBloc) 
         g=(currentPath->getPath().size())-sizeBloc;
      //insert the cells in the current bloc
      while(j<g+sizeBloc)
      {
	currentBloc->insertCell (map, currentBloc->getPath().size(), currentPath->getPath()[j]);
	j++;
      }
      if(j==g+sizeBloc) 
      {
	g=g+sizeBloc;
	j=g;
      } 
    currentBloc->setCost(currentBloc->getPathCost(map,true));  
    
    
    //Swap, Insert et remove on a a bloc   
    for (uint i=1; i<currentBloc->getPath().size()-1; i++) //for each cell (except the start and the goal cells of the bloc: to maintain the feasiblity of the path)
     {
		//Make the swap Move 
		unvisitedCommonNeighbors=findUnvisitedCommonNeighborsInBloc(map,currentBloc,bestBlocs,currentBloc->getPath()[i-1],currentBloc->getPath()[i+1]);  
		for(uint j=0; j<unvisitedCommonNeighbors.size();j++)
		  { 
		    if (unvisitedCommonNeighbors[j]!=currentBloc->getPath()[i])
		    {
		      bool aspirationCriteria=false;
		      Path* newPathAfterSwap=swapMove(map,currentBloc,i, unvisitedCommonNeighbors[j],aspirationCriteria);	    
		      if ((newPathAfterSwap->getPathCost(map,false)!= currentBloc->getPathCost(map,false)) || ( (newPathAfterSwap->getPathCost(map,false) == currentBloc->getPathCost(map,false)) && newPathAfterSwap!=currentBloc))
		      {
			  Move swap;
			    swap.newPathAfterMove=newPathAfterSwap;
			    swap.previousCell=currentBloc->getPath()[i];
			    swap.position=i;
			    swap.moveType="Swap";
			    swap.aspirationCriteria=aspirationCriteria;
			  neighborhoodBloc.push_back(swap);
			 // currentBloc=newPathAfterSwap;
		      }
		    }
		  }
		//Make Insert Move 
		unvisitedCommonNeighbors.clear();
		unvisitedCommonNeighbors=findUnvisitedCommonNeighborsInBloc(map,currentBloc,bestBlocs,currentBloc->getPath()[i-1],currentBloc->getPath()[i]);  
		for(uint j=0; j<unvisitedCommonNeighbors.size();j++)
		{
		  bool aspirationCriteria=false;
		  Path* newPathAfterInsert=insertMove(map,currentBloc,i, unvisitedCommonNeighbors[j],aspirationCriteria);
		  if ((newPathAfterInsert->getPathCost(map,false) != currentBloc->getPathCost(map,false)) || ( (newPathAfterInsert->getPathCost(map,false) == currentBloc->getPathCost(map,false)) && newPathAfterInsert!=currentBloc))
		  {
		        Move insert;
			insert.newPathAfterMove=newPathAfterInsert;
			insert.previousCell=currentBloc->getPath()[i];
			insert.position=i;
			insert.moveType="Insert";
			insert.aspirationCriteria=aspirationCriteria;
			// cout << "aspirationCriteria insert= " << aspirationCriteria << endl;
		      neighborhoodBloc.push_back(insert);
		      // currentBloc=newPathAfterInsert;
		  }
		}
		//Make Remove Move
		vector<int> neighborCells=findFreeNeighborCell (map,currentBloc->getPath()[i-1]); 
		if(contains(currentBloc->getPath()[i+1], neighborCells))
		{
		  bool aspirationCriteria=false;
		  Path* newPathAfterRemove=removeMove(map,currentBloc,i,aspirationCriteria);
		  if ((newPathAfterRemove->getPathCost(map,false) != currentBloc->getPathCost(map,false)) || ( (newPathAfterRemove->getPathCost(map,false) == currentBloc->getPathCost(map,false)) && newPathAfterRemove!=currentBloc))
		  {
		    Move remove;
		      remove.newPathAfterMove=newPathAfterRemove;
		      remove.previousCell=currentBloc->getPath()[i];
		      remove.position=i;
		      remove.moveType="Remove";
		      remove.aspirationCriteria=aspirationCriteria;
		    neighborhoodBloc.push_back(remove);
		    currentBloc = newPathAfterRemove;
	
		   }
		}
      
      }
      
      //choose the best bloc
      if(neighborhoodBloc.size() != 0 )
      { 
	  //Search the best neighbor
	  int indexBestBlocAfterOneIteration=findBestNeighbor(map,neighborhoodBloc);
	  //make the best move of one bloc tabu
	  makeTheMoveTabu (neighborhoodBloc, indexBestBlocAfterOneIteration,TLIN,TLOUT, NITS);
	  //put the best bloc in the vector bestPaths
	  bestBlocs.push_back(neighborhoodBloc[indexBestBlocAfterOneIteration].newPathAfterMove);
	}
	else
	  bestBlocs.push_back(currentBloc); //if there is no change, so the best bloc is the current bloc
	
    currentBloc->getPath().clear(); // clear the current bloc
    
    } 
    
    //construct the best path from the vector bestBlocs   
    Path* bestPathInOneIteration=new Path();
    for (int i=0; i<bestBlocs.size(); i++)
    {
      for(int j=0; j<bestBlocs[i]->getPath().size(); j++)
      {
	bestPathInOneIteration->insertCell(map,bestPathInOneIteration->getPath().size(),bestBlocs[i]->getPath()[j]);
      }
    }
    
    //put the best path in the vector bestPaths
    bestPaths.push_back(bestPathInOneIteration);
    
    //update the tabu list
     updateTabuLists(TLIN, TLOUT, NITS);
     setTabuListIn(TLIN);
     setTabuListOut(TLOUT);
    
       
    //change the current path
     currentPath=bestPathInOneIteration;
    
     clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
     timespec executionTime=diff(time1,time2); 
     bestExecutionTimes.push_back(executionTime);
        
     cout<<"Current Best Path Cost: "<<currentPath->getPathCost(map,false) <<"at iteration "<< NITS << "time=" <<  (diff(time1,time2).tv_nsec)*1e-6  << "size path= "<< currentPath->getPath().size()<<endl;
   
    //increment NITS
      NITS++;
    
  }
  //search the bestPath in the vector bestPaths
  if(bestPaths.size()!=0)
  {  
     int indexBestPath=findBestPath(map,bestPaths);  
     bestExecutionTime=bestExecutionTimes[indexBestPath] ;
     return bestPaths[indexBestPath];
  }
  else 
    return initialPath;

}
/*******************************************************************************/
//Function Name: updateTabuLists
//Inputs: 
//Output: 
//Description: it is used to update the tabu list in and out after an interation
/*********************************************************************************/
void TabuSearch::updateTabuLists(list<TabuMove> & TLIN, list<TabuMove> & TLOUT, int iteration)
{
  list<TabuMove> listIn= TLIN;
  list<TabuMove> listOut= TLOUT;
  
  //update TabuListIn
  TLIN.clear();
  if (listIn.size()!=0)   
    for (list<TabuMove>::iterator it = listIn.begin(); it !=listIn.end(); it++)
      if((*it).getDateOfExpiration()!=iteration)
	TLIN.push_back(*it);
  
  //update TabuListOut
  TLOUT.clear();
  if (listOut.size()!=0) 
    for (list<TabuMove>::iterator it =listOut.begin(); it !=listOut.end(); it++)
       if((*it).getDateOfExpiration()!=iteration)
	TLOUT.push_back(*it);   
}
/*******************************************************************************/
//Function Name: makeTheMoveTabu
//Inputs: 
//Output: 
//Description: it is used to 
/*********************************************************************************/
void TabuSearch:: makeTheMoveTabu (vector <Move> newPaths, int indexBestPathAfterOneIteration,list<TabuMove> & TLIN, list<TabuMove> & TLOUT, int iteration)
{
      
    if(newPaths[indexBestPathAfterOneIteration].moveType=="Swap")
      {
	addTabuMove(newPaths[indexBestPathAfterOneIteration].newPathAfterMove->getPath()[(newPaths[indexBestPathAfterOneIteration].position)-1], newPaths[indexBestPathAfterOneIteration].newPathAfterMove->getPath()[newPaths[indexBestPathAfterOneIteration].position],TLIN, iteration,newPaths[indexBestPathAfterOneIteration].aspirationCriteria);
        addTabuMove( newPaths[indexBestPathAfterOneIteration].newPathAfterMove->getPath()[newPaths[indexBestPathAfterOneIteration].position],newPaths[indexBestPathAfterOneIteration].newPathAfterMove->getPath()[(newPaths[indexBestPathAfterOneIteration].position)+1],TLIN,iteration,newPaths[indexBestPathAfterOneIteration].aspirationCriteria);
        addTabuMove(newPaths[indexBestPathAfterOneIteration].newPathAfterMove->getPath()[(newPaths[indexBestPathAfterOneIteration].position)-1],newPaths[indexBestPathAfterOneIteration].previousCell,TLOUT,iteration,newPaths[indexBestPathAfterOneIteration].aspirationCriteria);
        addTabuMove(newPaths[indexBestPathAfterOneIteration].previousCell,newPaths[indexBestPathAfterOneIteration].newPathAfterMove->getPath()[(newPaths[indexBestPathAfterOneIteration].position)+1],TLOUT,iteration,newPaths[indexBestPathAfterOneIteration].aspirationCriteria); 
      }
      else
      {
	  if(newPaths[indexBestPathAfterOneIteration].moveType=="Insert")
	  { 
	    addTabuMove(newPaths[indexBestPathAfterOneIteration].newPathAfterMove->getPath()[(newPaths[indexBestPathAfterOneIteration].position)-1], newPaths[indexBestPathAfterOneIteration].newPathAfterMove->getPath()[newPaths[indexBestPathAfterOneIteration].position],TLIN,iteration,newPaths[indexBestPathAfterOneIteration].aspirationCriteria);
	    addTabuMove(newPaths[indexBestPathAfterOneIteration].newPathAfterMove->getPath()[newPaths[indexBestPathAfterOneIteration].position],newPaths[indexBestPathAfterOneIteration].previousCell,TLIN,iteration,newPaths[indexBestPathAfterOneIteration].aspirationCriteria);
	    addTabuMove(newPaths[indexBestPathAfterOneIteration].newPathAfterMove->getPath()[(newPaths[indexBestPathAfterOneIteration].position)-1],newPaths[indexBestPathAfterOneIteration].newPathAfterMove->getPath()[newPaths[indexBestPathAfterOneIteration].position+1],TLOUT,iteration,newPaths[indexBestPathAfterOneIteration].aspirationCriteria);
	  }
	  else
	  {
	      if(newPaths[indexBestPathAfterOneIteration].moveType=="Remove")
	      {
		addTabuMove(newPaths[indexBestPathAfterOneIteration].newPathAfterMove->getPath()[(newPaths[indexBestPathAfterOneIteration].position)-1], newPaths[indexBestPathAfterOneIteration].previousCell,TLOUT,iteration,newPaths[indexBestPathAfterOneIteration].aspirationCriteria);
               addTabuMove(newPaths[indexBestPathAfterOneIteration].previousCell,newPaths[indexBestPathAfterOneIteration].newPathAfterMove->getPath()[(newPaths[indexBestPathAfterOneIteration].position)],TLOUT,iteration,newPaths[indexBestPathAfterOneIteration].aspirationCriteria);
               addTabuMove(newPaths[indexBestPathAfterOneIteration].newPathAfterMove->getPath()[(newPaths[indexBestPathAfterOneIteration].position)-1],newPaths[indexBestPathAfterOneIteration].newPathAfterMove->getPath()[(newPaths[indexBestPathAfterOneIteration].position)],TLIN,iteration,newPaths[indexBestPathAfterOneIteration].aspirationCriteria);
	      }
	  }
        }
}
/*******************************************************************************/
//Function Name: diversification
//Inputs: 
//Output: 
//Description: 
/*********************************************************************************/
/*Path* TabuSearch:: diversification(OccupancyGridMap* map,int radius,int startCell, int goalCell)
{
    //Step 1. Draw the straight line between S and G
    Path* straightLine=drawStraightLineStartGoal (map,startCell,goalCell);
    //Setp 2: select the point near the center point of the straight line
    int index = (straightLine->getPath().size())/2 -1;
    int centerCell=straightLine->getPath()[index];
  
    //Step 3: find all free neighbors the selected center point with a radius of n cells, n is a paramter
    vector <int> neighbors=getNeighborsAtRadiusN(map,radius,centerCell);
    if(neighbors.size()!=0)
    {
    //Step 4: select one neighbor found in Step 3 and use it as cross point A
    int indexCrossPoint=rand()%(neighbors.size());
    int crossPoint=neighbors[indexCrossPoint];
  
    //Step 5: find a path between S and A
    Path* pathFromStartToCrossPoint=getInitialPath(map,startCell, crossPoint, true);
 
    //Step 6: find a path between Aand G
    Path* pathFromCrossPointToGoal=getInitialPath(map,crossPoint, goalCell, true);
   
    //Step 7: construct the final Path
    Path* newPath=new Path();
    newPath->setPath(pathFromStartToCrossPoint->getPath());
    
    for(uint j=1; j<pathFromCrossPointToGoal->getPath().size(); j++)
    newPath->insertCell(map,newPath->getPath().size(),pathFromCrossPointToGoal->getPath()[j]);
    newPath->setCost(pathFromStartToCrossPoint->getPathCost(map,false)+pathFromCrossPointToGoal->getPathCost(map,false));
    
    return newPath;
    }
    else
    {
      cout << "please choose a greater radius" << endl;
      exit(1);
    }
}*/
