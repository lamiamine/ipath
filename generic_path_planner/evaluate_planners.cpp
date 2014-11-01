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
#include"GenericPathPlanner.h"
#include <iomanip>
#include <sstream>

ofstream MyExcelFile;


int clock_gettime(clockid_t clk_id, struct timespect *tp);

timespec diff(timespec start, timespec end)
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


void chooseRandomStartAndGoalLocations(OccupancyGridMap* OGM, int & startCell, int & goalCell){
  cout<<"choose"<<endl;
  //srand ( time(NULL) );
  //sleep(1.0);
  startCell = 0;
  do
  startCell=rand() % (OGM->getWidth()*OGM->getHeight());
  while(!OGM->isFree(startCell));
startCell=0;

  cout<<"start: "<<startCell<<endl;

  goalCell =  0;
  do
  goalCell =rand() % (OGM->getWidth()*OGM->getHeight());
  while(!OGM->isFree(goalCell));

   cout<<"goal: "<<goalCell<<endl;
  /* we may add some more conditions to make sure that start and goal are enough far from each other*/

}

int evaluate_planner (GenericPathPlanner* pathPlannerGreedy, OccupancyGridMap* OGM, int startCell, int goalCell,string statistics_file, string file,string sourcePath){

  int index=file.find("/", 0);
  string fileName=file.substr(index+1);

  stringstream ss;
  ss << startCell;
  ss << "_";
  ss<<goalCell;
  string str = ss.str();

  string best="path_output/Greedy_search-"+str+"_"+fileName;
  const char* fileBest = best.c_str();


	 cout <<"****************** Start Greedy search ******************" << endl;
	 Path* bestPath= new Path();
	   timespec time1, time2;
	   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);
	 bestPath=pathPlannerGreedy->getInitialPath(OGM, startCell, goalCell, false);
	   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);

	 bestPath->setName("best path");
	 bestPath->printPath();
	 cout<<endl;
	 cout << "best path cost: " << bestPath->getPathCost(OGM, true) << endl;
	 cout<<endl;
	   cout<<"time to generate best path= " << (diff(time1,time2).tv_nsec)*1e-6<< "microseconds" << endl;

	 OccupancyGridMap* OGMBestPath = new OccupancyGridMap();
	 OGMBestPath->importMapLayout(sourcePath,file.c_str());
	 bestPath->exportPath(OGMBestPath, fileBest);


	  MyExcelFile <<"\t";
	  MyExcelFile << OGM->getWidth() <<"\t"<<OGM->getHeight()<<"\t";
	  MyExcelFile <<OGM->getObstacleRatio()<<"\t"<<OGM->getObstacleSize()<<"\t";
	  MyExcelFile << startCell <<"\t"<<goalCell<<"\t"<<bestPath->getCost()<<"\t";
	  MyExcelFile <<(diff(time1,time2).tv_nsec)*1e-6<<" ms" <<"\t";
	  MyExcelFile <<(diff(time1,time2).tv_sec) <<" s" << endl;

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
	Path* bestPath= new Path();
	string resultPath="../bin/";
	string mapFileName;
	string mapFilePath;
	string sourcePath="../map_folder/";
	string statisticFileName;
	statisticFileName = "GreedySearchStatistics.ods";



     /*****************************maps 10*10***********************************************/
    mapFileArray.push_back("map-W10-H10/W10-H10-ObRatio0.1-ObSize1-Nbr01.pgm");//Ok
    // mapFileArray.push_back("map-W10-H10/W10-H10-ObRatio0.2-ObSize2-Nbr10.pgm");
    // mapFileArray.push_back("map-W10-H10/W10-H10-ObRatio0.5-ObSize2-Nbr01.pgm");
    
     /*****************************maps 20*20***********************************************/
      //mapFileArray.push_back("map-W20-H20/W20-H20-ObRatio0.1-ObSize1-Nbr01.pgm");//Ok
      // mapFileArray.push_back("map-W20-H20/W20-H20-ObRatio0.2-ObSize2-Nbr01.pgm");
    // mapFileArray.push_back("map-W20-H20/W20-H20-ObRatio0.3-ObSize3-Nbr01.pgm");
    
     /*****************************maps 30*30***********************************************/
  // mapFileArray.push_back("map-W30-H30/W30-H30-ObRatio0.1-ObSize1-Nbr01.pgm");//Ok
   //  mapFileArray.push_back("map-W30-H30/W30-H30-ObRatio0.2-ObSize2-Nbr01.pgm");//Ok
     //  mapFileArray.push_back("map-W30-H30/W30-H30-ObRatio0.3-ObSize3-Nbr01.pgm");//Ok
     //   mapFileArray.push_back("map-W30-H30/ W30-H30-ObRatio0.3-ObSize5-Nbr01.pgm");//Ok
         
         /*****************************maps 50*50***********************************************/
     //  mapFileArray.push_back("map-W50-H50/W50-H50-ObRatio0.1-ObSize1-Nbr01.pgm");//Ok
       //  mapFileArray.push_back("map-W50-H50/W50-H50-ObRatio0.2-ObSize5-Nbr01.pgm");//Ok
      // mapFileArray.push_back("map-W50-H50/W50-H50-ObRatio0.3-ObSize3-Nbr01.pgm");//Ok 
     
     /*****************************maps 60*60***********************************************/
        //mapFileArray.push_back("map-W60-H60/W60-H60-ObRatio0.1-ObSize3-Nbr01.pgm");//Ok
    //      mapFileArray.push_back("map-W60-H60/W60-H60-ObRatio0.3-ObSize3-Nbr01.pgm");//Ok
        // mapFileArray.push_back("map-W60-H60/W60-H60-ObRatio0.3-ObSize5-Nbr01.pgm");//Ok    
	
        
   /******************************maps 100*100********************************************/
	//mapFileArray.push_back("map-W100-H100/W100-H100-ObRatio0.1-ObSize10-Nbr01.pgm");//Ok
 	//mapFileArray.push_back("map-W100-H100/W100-H100-ObRatio0.2-ObSize2-Nbr01.pgm");//Ok
 	//mapFileArray.push_back("map-W100-H100/W100-H100-ObRatio0.2-ObSize5-Nbr01.pgm");//No
        // mapFileArray.push_back("map-W100-H100/W100-H100-ObRatio0.3-ObSize2-Nbr01.pgm");//Ok
       //  mapFileArray.push_back("map-W100-H100/W100-H100-ObRatio0.4-ObSize10-Nbr01.pgm");//Ok
         //   mapFileArray.push_back("map-W100-H100/W100-H100-ObRatio0.4-ObSize2-Nbr01.pgm");
	 //   mapFileArray.push_back("map-W100-H100/W100-H100-ObRatio0.4-ObSize5-Nbr01.pgm");
           //  mapFileArray.push_back("map-W100-H100/W100-H100-ObRatio0.25-ObSize10-Nbr01.pgm");
	  //   mapFileArray.push_back("map-W100-H100/W100-H100-ObRatio0.3703-ObSize2-Nbr01.pgm");
			/******************************maps 500*500********************************************/
//	mapFileArray.push_back("map-W500-H500/W500-H500-ObRatio0.1-ObSize5-Nbr01.pgm");//OK
  //    mapFileArray.push_back("map-W500-H500/W500-H500-ObRatio0.2-ObSize5-Nbr01.pgm");//OK
  //mapFileArray.push_back("map-W500-H500/W500-H500-ObRatio0.2-ObSize20-Nbr01.pgm");//OK
   // mapFileArray.push_back("map-W500-H500/W500-H500-ObRatio0.2-ObSize50-Nbr01.pgm");// optimal from initial path
   // mapFileArray.push_back("map-W500-H500/W500-H500-ObRatio0.3-ObSize5-Nbr01.pgm");//OK
    // mapFileArray.push_back("map-W500-H500/W500-H500-ObRatio0.3-ObSize20-Nbr01.pgm");//OK
    //mapFileArray.push_back("map-W500-H500/W500-H500-ObRatio0.3-ObSize50-Nbr01.pgm");//Ok
   //mapFileArray.push_back("map-W500-H500/W500-H500-ObRatio0.4-ObSize5-Nbr01.pgm");//Ok No
  //mapFileArray.push_back("map-W500-H500/W500-H500-ObRatio0.4-ObSize20-Nbr01.pgm");//No
   //  mapFileArray.push_back("map-W500-H500/W500-H500-ObRatio0.4-ObSize50-Nbr01.pgm");//Ok
   // mapFileArray.push_back("map-W500-H500/W500-H500-ObRatio0.4-ObSize5-Nbr01.pgm");//Ok

/******************************maps 1000*1000********************************************/
   // mapFileArray.push_back("map-W1000-H1000/W1000-H1000-ObRatio0.2-ObSize20-Nbr01.pgm");//initial solution 
   //  mapFileArray.push_back("map-W1000-H1000/W1000-H1000-ObRatio0.2-ObSize50-Nbr01.pgm");//No
     //mapFileArray.push_back("map-W1000-H1000/W1000-H1000-ObRatio0.3-ObSize20-Nbr01.pgm");//initial solution 
   //mapFileArray.push_back("map-W1000-H1000/W1000-H1000-ObRatio0.3-ObSize50-Nbr01.pgm");//No
   //  mapFileArray.push_back("map-W1000-H1000/W1000-H1000-ObRatio0.4-ObSize20-Nbr01.pgm");//No
    //mapFileArray.push_back("map-W1000-H1000/W1000-H1000-ObRatio0.4-ObSize50-Nbr01.pgm");//No

/******************************maps 2000*2000********************************************/
      //mapFileArray.push_back("map-W2000-H2000/W2000-H2000-ObRatio0.2-ObSize50-Nbr01.pgm");//Ok
    //mapFileArray.push_back("map-W2000-H2000/W2000-H2000-ObRatio0.2-ObSize100-Nbr01.pgm");//OK
 	//mapFileArray.push_back("map-W2000-H2000/W2000-H2000-ObRatio0.3-ObSize50-Nbr01.pgm");//
 			//mapFileArray.push_back("map-W2000-H2000/W2000-H2000-ObRatio0.3-ObSize100-Nbr01.pgm");//No
	// mapFileArray.push_back("map-W2000-H2000/W2000-H2000-ObRatio0.4-ObSize100-Nbr01.pgm");//No
	//	 mapFileArray.push_back("map-W2000-H2000/W2000-H2000-ObRatio0.5-ObSize50-Nbr01.pgm");//No

			
                       /*add as many as you need*/

	MyExcelFile.open(statisticFileName.c_str(), ios::trunc);
	MyExcelFile << "A*\tmapWidth\tmapHeight\tObstacleRatio\tObstacleSize\tstartCell\tGoalCell\tBestA*PathCost\tA*ExecTimePath(ms)\tA*ExecTimePath(s)  "  << endl;
	cout << "starting the execution of the Greedy search algorithm ..." << endl;
	for (uint i=0; i<mapFileArray.size(); i++){

		//get the map from the array of maps
		cout <<"map name: " << mapFileArray[i]<<endl;

		//  for (int count=0; count<20; count++){
		OccupancyGridMap* OGM = new OccupancyGridMap();
		OGM->importMapLayout(sourcePath, mapFileArray[i].c_str());

		
		GenericPathPlanner* pathPlannerGreedy = new GenericPathPlanner();


		cout<<endl<<"scenario 1 "<<endl;
		chooseRandomStartAndGoalLocations(OGM, startCell, goalCell);
		for (int run=0; run<1;run++){
		cout<<"*****************************run "<<run<<"**********************************"<<endl;
			int j=evaluate_planner (pathPlannerGreedy, OGM, startCell, goalCell, statisticFileName,mapFileArray[i],sourcePath);
		cout<<"******************************************************************************"<<endl;
		}
	}
    return 0;
}
