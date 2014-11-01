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

#include "evaluatePlanners.h"
#include <unistd.h>
 

/**********************************************************/
//Function: Constructor
/**********************************************************/
template <typename T>
evaluatePlanners<T>::evaluatePlanners()
{
  
}
template <typename T>
evaluatePlanners<T>::evaluatePlanners(string statFile)
{
   stats_file=statFile;
}
/**********************************************************/
//Function: destructor
/**********************************************************/
template <typename T>
evaluatePlanners <T>::~evaluatePlanners(void){
}
/**********************************************************/
//Function: Mutators
/**********************************************************/
template <typename T>
void evaluatePlanners <T>::setStatFile(string statFile)
{
  stats_file=statFile;
}
/**********************************************************/
//Function: Accessors
/**********************************************************/
template <typename T>
string evaluatePlanners<T>::getStatFile()
{
  return stats_file;
}
/*******************************************************************************/
//Function Name: run
//Inputs: 
//Output: 
//Description: this function is responsible to run simulation for one scenario for a given planner 
/*********************************************************************************/
template <typename T>
void evaluatePlanners<T>::run(T* pathPlanner, OccupancyGridMap* OGM, int startCell, int goalCell)
{
        ofstream excel_File;
        excel_File.open(getStatFile().c_str(), ios::app); 
        Path* bestPath= new Path();
	timespec time1, time2, time3;
	 
		
        TabuSearch* TSplanner = dynamic_cast<TabuSearch*>(pathPlanner);
        AStar* AStarplanner = dynamic_cast<AStar*>(pathPlanner);
	GA* GAplanner = dynamic_cast<GA*>(pathPlanner);
	RAStar* RAStarplanner = dynamic_cast<RAStar*>(pathPlanner);
	
	/****************Tabu Search planner*******************/
        if( TSplanner )
 	{
 	   Path* initialPath= new Path();
           initialPath = TSplanner->getInitialPath(OGM, startCell, goalCell, true); 
	   if (initialPath->getPath().size()==0)
	     cout << "Cannot generate the initial path please choose other start and goal! " << endl;
	     // we should try another method to generate the intial path
	     
	   else
            bestPath=TSplanner->findPath(OGM,initialPath, time1);
 	}
	/***************AStar****************************/
        if( AStarplanner )
	{
	   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
 	   bestPath=AStarplanner->findPath(OGM, startCell, goalCell);
	   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time3);
	   time1=diff(time2,time3);
	}
        /**************GAplanner********************************/
	if(GAplanner)
	{
	  Population* initialPopulation = new Population();
	  initialPopulation = GAplanner->getInitialPopulation(OGM, startCell,goalCell);
	  if (initialPopulation->getPopulation().size() > 0) 
	  {
	     bestPath=GAplanner->findPath(OGM, initialPopulation, time1);
	  }
	  else
	    cout<<"Can not generate initial population, please choose different start or goal positions"<<endl;
	}
	/*************RAStar*************************************/
	if(RAStarplanner)
	{
	  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
	  bestPath=RAStarplanner->findPath(OGM, startCell,goalCell);	
	  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time3);
	  time1=diff(time2,time3);
	}

	
 if (bestPath->getPath().size()!=0)
 { 
     excel_File << startCell <<"\t"<<goalCell<<"\t"<<bestPath->getPath().size()<<"\t"<<bestPath->getPathCost(OGM,true)<<"\t"<<(time1.tv_nsec)*1e-6 << endl;
  }
}
/*******************************************************************************/
//Function Name: run (pathPlanner)
//Inputs: the path planner
//Output: 
//Description:this function is used to run simulation for a set scenarios for a given planner  
/*********************************************************************************/
template <typename T>
void evaluatePlanners<T>::run(T* pathPlanner, OccupancyGridMap* OGM, int** startGoalConfigArray, int numberOfScenarios, int numberOfruns)
{

    ofstream excelFile;
    excelFile.open(getStatFile().c_str(), ios::app); 
    excelFile << "mapWidth" << " " << OGM->getWidth() << endl;
    excelFile << "mapHeight" << " " << OGM->getHeight() << endl;
    excelFile << "ObstacleRatio" << " " << OGM->getObstacleRatio()<< endl;
    excelFile << "ObstacleSize" << " " << OGM->getObstacleSize() << endl;
    excelFile <<endl;
    excelFile <<endl;
    
    excelFile << "startCell" << "\t" << "GoalCell"<< "\t" << "BestPathSize" << "\t" << "BestPathCost" << "\t" << "ExecTimeBestPath" <<  endl;
    //execute the planner for the different start and goal cells
    for(int j=0; j < numberOfScenarios ;j++)
    {
       for (int h=0; h < numberOfruns; h++) 
	 run(pathPlanner,OGM, startGoalConfigArray[j][0],startGoalConfigArray[j][1]);
       
       excelFile << endl;
       excelFile << endl;
    }
}
/*******************************************************************************/
//Function Name: clock_gettime and diff
//Inputs: 
//Output: 
//Description: these two functions are responsible to calculate the excution time of the planner
/*********************************************************************************/
int clock_gettime(clockid_t clk_id, struct timespec *tp);
template <typename T>
timespec evaluatePlanners<T>::diff(timespec start, timespec end)
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


