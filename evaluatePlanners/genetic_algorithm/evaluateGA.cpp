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
#include "../../evaluatePlanners/evaluatePlanners.cpp"
#include <iomanip>
#include <math.h>

using namespace std;  
unsigned int sleep(unsigned int seconds);
/*******************************************************************************/
//Function Name: loadMaps 
//Inputs: 
//Output: 
//Description: 
/*********************************************************************************/
void loadMaps(string mapFile, vector <string> & mapFileArray)
{
  ifstream infile(mapFile.c_str());
  string tmp;
 
   
  if(!infile) 
  {      
    //file couldn't be opened
    cerr << "[I/O Error] The map file could not be opened. \nCheck file name and/or path" << endl;
    exit(1);
  }
  else
  {
    while(getline(infile,tmp))
    {
      mapFileArray.push_back(tmp);
    }
    infile.close();
  }
}
/*******************************************************************************/
//Function Name: chooseFarStartAndGoalLocations 
//Inputs: the occupancy grid map
//Output: 
//Description: this function is used to choose randomly start and goal positions and to check their validity
/*********************************************************************************/
void chooseRandomStartAndGoalLocations(OccupancyGridMap* OGM,GenericPathPlanner* GPP,int count, int** startGoalConfigArray)
{
  int startCell;
  int goalCell;
  
  srand ( time(NULL) );
  sleep(1.0);
  
  startCell = 0;
  goalCell =  0;
  
  do
  {
  startCell=rand() % (OGM->getWidth()*OGM->getHeight());
  goalCell =rand() % (OGM->getWidth()*OGM->getHeight());
  }while(!GPP->isStartAndGoalCellsValid(OGM,startCell,goalCell)); 
  
  startGoalConfigArray[count][0]=startCell;
  startGoalConfigArray[count][1]=goalCell;
}
/*******************************************************************************/
//Function Name: chooseFarStartAndGoalLocations 
//Inputs: the occupancy grid map
//Output: 
//Description: this function is used to choose randomly start and goal positions and to check their validity
/*********************************************************************************/
void chooseFarStartAndGoalLocations(OccupancyGridMap* OGM,GenericPathPlanner* GPP, int thresholdDistance, int count, int** startGoalConfigArray)
{
  int startCell;
  int goalCell;
  
  srand ( time(NULL) );
  sleep(1.0);
  
  startCell = 0;
  goalCell =  0;
  
  do
  {
  startCell=rand() % (OGM->getWidth()*OGM->getHeight());
  goalCell =rand() % (OGM->getWidth()*OGM->getHeight());
  }while((!GPP->isStartAndGoalCellsValid(OGM,startCell,goalCell)) | (((OGM->getCellRowID(goalCell)-OGM->getCellRowID(startCell))+(OGM->getCellColID(goalCell)-OGM->getCellColID(startCell))) > thresholdDistance)); 
  
  startGoalConfigArray[count][0]=startCell;
  startGoalConfigArray[count][1]=goalCell;
}
/*******************************************************************************/
//Function Name:  
//Inputs: the occupancy grid map
//Output: 
//Description: 
/*********************************************************************************/
void longestStraightPath(OccupancyGridMap* OGM, int** startGoalConfigArray)
{
  startGoalConfigArray[0][0]=0;
  startGoalConfigArray[0][1]=(OGM->getWidth()*OGM->getHeight())-1;
  
}

void chooseStartAndGoalCells( OccupancyGridMap* OGM, int** startGoalConfigArray, int numberOfScenarios)
{
  // the generic path planner    
  GenericPathPlanner* GPP=new GenericPathPlanner();
  
  //longestStraightPath
   longestStraightPath(OGM, startGoalConfigArray);
    
  //choose randomly 15 start and goal positions for the current map_file
  for (int j=0; j<numberOfScenarios/2; j++)
    chooseRandomStartAndGoalLocations(OGM,GPP,j,startGoalConfigArray);
    
  //choose randomly 15 start and goal positions with constraints
  int thresholdDistance= ((OGM->getCellRowID((OGM->getWidth()*OGM->getHeight())-1)-OGM->getCellRowID(0))+(OGM->getCellColID((OGM->getWidth()*OGM->getHeight())-1)-OGM->getCellColID(0)))/2;  
  for (int j=numberOfScenarios/2; j<numberOfScenarios; j++)
    chooseFarStartAndGoalLocations(OGM,GPP, thresholdDistance,j,startGoalConfigArray); 
}


int main(){
  
  string mapFile="../bin/mapFile";
  
  string sourcePath="../../../map_folder/";
  
  vector <string> mapFileArray;
  int** startGoalConfigArray;
  
  // load all maps in the array
  loadMaps(mapFile, mapFileArray);

  int numberOfScenarios = 2; // the number of the scenarios (start and goal positions)
  int numberOfruns = 10; // how many times you want to repeat each scenario

  // create the GA planner

  evaluatePlanners <GA>* GAplanner=new evaluatePlanners <GA> (); 

  // GA parameters
  int numberOfIterations = 15;
  uint populationSize = 15;
  int crossoverType = 1; // 1: one point, 2: two point, 3: modified.
  float crossoverProbability = 0.9;
  float mutationProbability = 0.01;
  int mutationIterationNumber = 50;
  float minInitialPathCost = 0;
  int radius = 2;

  GA* ga=new GA(numberOfIterations, populationSize, crossoverType, crossoverProbability, mutationProbability, mutationIterationNumber, minInitialPathCost, radius);
  GAplanner->setStatFile("../bin/GAStatistics.xlsx");

  // for each map 
  for (int i=0; i<mapFileArray.size(); i++)
  {
     // get the map name
      int indexFirstSpace=mapFileArray[i].find(" ", 0);
      string fileName=mapFileArray[i].substr(indexFirstSpace+1, strlen(mapFileArray[i].c_str()));
  
     //read the occupancy grid map
     OccupancyGridMap* OGM = new OccupancyGridMap();
     OGM->importMapLayout(sourcePath, fileName.c_str());
     
     // create the start and goal array
     startGoalConfigArray = (int**) malloc(sizeof( int*)*numberOfScenarios);
     for(int j = 0; j < numberOfScenarios; j++)
     startGoalConfigArray[j] = (int*) malloc(sizeof(int)*2);
   
         
    // choose 30 start and goal cells and run the different planners.
     chooseStartAndGoalCells(OGM, startGoalConfigArray, numberOfScenarios);
  
    // run GA
  
      cout << "****************GENETIC_ALGORITHM******************"<< endl;
      GAplanner->run(ga, OGM, startGoalConfigArray, numberOfScenarios, numberOfruns);
 
   
  }
  
return 0;
}
