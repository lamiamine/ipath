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


#include <stdio.h>
#include <cstdio>
#include <cstdlib>
#include<iostream>
#include <fstream>
#include <stdlib.h>
#include <time.h>    
#include"ACO.h"
#include <iomanip>
#include <limits>



#include <algorithm> 
#include <math.h> 
#include <numeric>


ofstream MyExcelFile;

    
clock_t getCurrentTime()
{
  return clock();
}

double getDuration(clock_t startTime)
{ 
  return ((getCurrentTime()-startTime)/CLOCKS_PER_SEC*1000);
}

void chooseRandomStartAndGoalLocations(OccupancyGridMap* OGM, int &startCell, int &goalCell){
 
  srand ( time(NULL) );
  //sleep(1.0);
  startCell = 0;
  do
  startCell=rand() % (OGM->getWidth()*OGM->getHeight());
  while(!OGM->isFree(startCell));
  cout<<"start: "<<startCell<<endl;
  
  goalCell =  0;
  do
  goalCell =rand() % (OGM->getWidth()*OGM->getHeight());
  while(!OGM->isFree(goalCell));
   cout<<"goal: "<<goalCell<<endl;
  /* we may add some more conditions to make sure that start and goal are enough far from each other*/
  
}

int evaluate_planner (ACO* pathPlannerACO, OccupancyGridMap* OGM, int startCell, int goalCell,string statistics_file, string file,string sourcePath, bool enable_trace){
 
  
  /* take current time here */
  clock_t startTime;

  int index=file.find("/", 0);
  string fileName=file.substr(index+1);
  string best="path_output/BestPath-"+fileName;
  const char* fileBest = best.c_str();
   Path* bestPath= new Path();
   
   startTime=getCurrentTime();
   bestPath=pathPlannerACO->findPath(OGM,startCell, goalCell,enable_trace);
   double bestPathTime=getDuration(startTime); 
   bestPath->setName("best path"); 
   bestPath->printPath();
   cout << "best path cost: " << bestPath->getCost() << endl;
   cout<< "\ntime to find the best path =  " <<bestPathTime<< " milliseconds" <<endl;
	 
   OccupancyGridMap* OGMBestPath = new OccupancyGridMap(); 
   OGMBestPath->importMapLayout(sourcePath,file.c_str());	  		  
   bestPath->exportPath(OGMBestPath, fileBest);
   
   MyExcelFile << OGM->getWidth() <<"\t"<<OGM->getHeight()<<"\t"<<OGM->getObstacleRatio()<<"\t"<<OGM->getObstacleSize()<<"\t";
   MyExcelFile << startCell <<"\t"<<goalCell<<"\t"<<"\t"<<bestPath->getCost()<<"\t";
   MyExcelFile << pathPlannerACO->getNumberOfAnts() <<"\t"<<pathPlannerACO->getNumberOfIterations() << endl;
    
    
    
   return 1;
}


using namespace std;    
int main(){
     //create statistic file
     vector <string> statistics_file;
     //create a table of file names
     vector <string> mapFileArray;
     int startCell;
     int goalCell;
     string resultPath="../bin/";
     string mapFileName;
     string mapFilePath;
     string sourcePath="../map_folder/";
     ACO* pathPlannerACO;
     string statisticFileName;
     statisticFileName = "ACOStatistics.ods";
     bool enable_trace = true;

     
        
     
       /*****************************maps 10*10***********************************************/
  //   mapFileArray.push_back("map-W10-H10/W10-H10-ObRatio0.1-ObSize1-Nbr01.pgm");//Ok
    // mapFileArray.push_back("map-W10-H10/W10-H10-ObRatio0.2-ObSize2-Nbr10.pgm");
    // mapFileArray.push_back("map-W10-H10/W10-H10-ObRatio0.5-ObSize2-Nbr01.pgm");
    
     /*****************************maps 20*20***********************************************/
       mapFileArray.push_back("map-W20-H20/W20-H20-ObRatio0.1-ObSize1-Nbr01.pgm");//Ok
  //   mapFileArray.push_back("map-W20-H20/W20-H20-ObRatio0.2-ObSize2-Nbr01.pgm");
    // mapFileArray.push_back("map-W20-H20/W20-H20-ObRatio0.3-ObSize3-Nbr01.pgm");
    
     /*****************************maps 30*30***********************************************/
  // mapFileArray.push_back("map-W30-H30/W30-H30-ObRatio0.1-ObSize1-Nbr01.pgm");//Ok
   //  mapFileArray.push_back("map-W30-H30/W30-H30-ObRatio0.2-ObSize2-Nbr01.pgm");//Ok
     //  mapFileArray.push_back("map-W30-H30/W30-H30-ObRatio0.3-ObSize3-Nbr01.pgm");//Ok
         //  mapFileArray.push_back("map-W30-H30/ W30-H30-ObRatio0.3-ObSize5-Nbr01.pgm");//Ok
         
         /*****************************maps 50*50***********************************************/
        // mapFileArray.push_back("map-W50-H50/W50-H50-ObRatio0.1-ObSize1-Nbr01.pgm");//Ok
        // mapFileArray.push_back("map-W50-H50/W50-H50-ObRatio0.2-ObSize5-Nbr01.pgm");//Ok
      //  mapFileArray.push_back("map-W50-H50/W50-H50-ObRatio0.3-ObSize3-Nbr01.pgm");//Ok 
     
     /*****************************maps 60*60***********************************************/
   //mapFileArray.push_back("map-W60-H60/W60-H60-ObRatio0.1-ObSize3-Nbr01.pgm");//Ok
    //  mapFileArray.push_back("map-W60-H60/W60-H60-ObRatio0.3-ObSize3-Nbr01.pgm");//Ok
    //   mapFileArray.push_back("map-W60-H60/W60-H60-ObRatio0.3-ObSize5-Nbr01.pgm");//Ok    
	
        
   /******************************maps 100*100*****************************************
	 // mapFileArray.push_back("map-W100-H100/W100-H100-ObRatio0.1-ObSize10-Nbr01.pgm");//Ok
 	//mapFileArray.push_back("map-W100-H100/W100-H100-ObRatio0.2-ObSize2-Nbr01.pgm");//Ok
 	// mapFileArray.push_back("map-W100-H100/W100-H100-ObRatio0.2-ObSize5-Nbr01.pgm");//No
        //mapFileArray.push_back("map-W100-H100/W100-H100-ObRatio0.3-ObSize2-Nbr01.pgm");//Ok
        //mapFileArray.push_back("map-W100-H100/W100-H100-ObRatio0.4-ObSize10-Nbr01.pgm");//Ok
                       
			/******************************maps 500*500********************************************/
	//mapFileArray.push_back("map-W500-H500/W500-H500-ObRatio0.1-ObSize5-Nbr01.pgm");//OK
       //mapFileArray.push_back("map-W500-H500/W500-H500-ObRatio0.2-ObSize5-Nbr01.pgm");//OK
   //  mapFileArray.push_back("map-W500-H500/W500-H500-ObRatio0.2-ObSize20-Nbr01.pgm");//OK
   // mapFileArray.push_back("map-W500-H500/W500-H500-ObRatio0.2-ObSize50-Nbr01.pgm");//OK
   //mapFileArray.push_back("map-W500-H500/W500-H500-ObRatio0.3-ObSize5-Nbr01.pgm");//OK
    // mapFileArray.push_back("map-W500-H500/W500-H500-ObRatio0.3-ObSize20-Nbr01.pgm");//OK
    //mapFileArray.push_back("map-W500-H500/W500-H500-ObRatio0.3-ObSize50-Nbr01.pgm");//Ok
   //mapFileArray.push_back("map-W500-H500/W500-H500-ObRatio0.4-ObSize5-Nbr01.pgm");//Ok No
  //mapFileArray.push_back("map-W500-H500/W500-H500-ObRatio0.4-ObSize20-Nbr01.pgm");//No
   //mapFileArray.push_back("map-W500-H500/W500-H500-ObRatio0.4-ObSize50-Nbr01.pgm");//Ok

/******************************maps 1000*1000********************************************/
  //mapFileArray.push_back("map-W1000-H1000/W1000-H1000-ObRatio0.2-ObSize20-Nbr01.pgm");//initial solution 
  // mapFileArray.push_back("map-W1000-H1000/W1000-H1000-ObRatio0.2-ObSize50-Nbr01.pgm");//No
    //mapFileArray.push_back("map-W1000-H1000/W1000-H1000-ObRatio0.3-ObSize20-Nbr01.pgm");//initial solution 
   //mapFileArray.push_back("map-W1000-H1000/W1000-H1000-ObRatio0.3-ObSize50-Nbr01.pgm");//No
//      mapFileArray.push_back("map-W1000-H1000/W1000-H1000-ObRatio0.4-ObSize20-Nbr01.pgm");//No
    //mapFileArray.push_back("map-W1000-H1000/W1000-H1000-ObRatio0.4-ObSize50-Nbr01.pgm");//No

/******************************maps 2000*2000********************************************/
 //mapFileArray.push_back("map-W2000-H2000/W2000-H2000-ObRatio0.2-ObSize50-Nbr01.pgm");//Ok
 //mapFileArray.push_back("map-W2000-H2000/W2000-H2000-ObRatio0.2-ObSize100-Nbr01.pgm");//OK
 	//mapFileArray.push_back("map-W2000-H2000/W2000-H2000-ObRatio0.3-ObSize50-Nbr01.pgm");//
 			//mapFileArray.push_back("map-W2000-H2000/W2000-H2000-ObRatio0.3-ObSize100-Nbr01.pgm");//No
	// mapFileArray.push_back("map-W2000-H2000/W2000-H2000-ObRatio0.4-ObSize100-Nbr01.pgm");//No
		// mapFileArray.push_back("map-W2000-H2000/W2000-H2000-ObRatio0.5-ObSize50-Nbr01.pgm");//No

			
                       /*add as many as you need*/
			
		        MyExcelFile.open(statisticFileName.c_str(), ios::trunc);
			MyExcelFile << "mapWidth\tmapHeight\tObstacleRatio\tObstacleSize\tstartCell\tGoalCell\tNumberOfIterations  "  << endl;
		        cout << "starting the execution of the ACO algorithm ..." << endl;
		        for (uint i=0; i<mapFileArray.size(); i++){ 
		        
			  //get the map from the array of maps
			  cout <<"map name: " << mapFileArray[i]<<endl;
			  
			  OccupancyGridMap* OGM = new OccupancyGridMap();
			 
			  
			  OGM->importMapLayout(sourcePath, mapFileArray[i].c_str());	
			  pathPlannerACO = new ACO(0.05L,1L,5L,0.1L,100,60,100) ;
			  GenericPathPlanner* GPP= new GenericPathPlanner();
			  
			  //Create a new planner (example ACO)
		        //  chooseRandomStartAndGoalLocations(OGM, startCell, goalCell);
			   cout <<"select randomly start and goal positions" << endl;
			  bool isvalid=false;
			    do{
			      cout<<"Start Cell: ";
			      cin>>startCell;
			      cout<<endl;
			      cout<<"Goal Cell: ";
			      cin>>goalCell;
			      cout<<endl;
			    if (GPP->isStartAndGoalCellsValid(OGM,startCell,goalCell))
			    {
			      isvalid=true;
			    }
			    }while (isvalid == false);	
			  cout <<"start cell: " << startCell<< "\tgoal cell: "<< goalCell <<endl; 
			                       
			 for (int run=0; run<1;run++){
			   cout<<"*****************************run "<<run<<"**********************************"<<endl;
		           int j=evaluate_planner (pathPlannerACO, OGM, startCell, goalCell, statisticFileName ,mapFileArray[i],sourcePath,enable_trace);
			   cout<<"******************************************************************************"<<endl;}
                         
}      
    return 0;
    
  }
