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

#include "CrossPoint.h"


using namespace std;

//constructor and destructor
CrossPoint::CrossPoint(void){

}

CrossPoint::CrossPoint (int ID, int CellIndex1, int CellIndex2){
	setCellID(ID);
	setPath1CellIndex(CellIndex1);
	setPath2CellIndex(CellIndex2);
}

CrossPoint::~CrossPoint(void){

}

//Define Mutators
void CrossPoint::setCellID(int ID){
	cellID=ID;
}
void CrossPoint::setPath1CellIndex (int CellIndex){
	path1CellIndex=CellIndex;
}
void CrossPoint::setPath2CellIndex (int CellIndex){
	path2CellIndex=CellIndex;
}

//Define Accessors
int CrossPoint::getCellID(){
	return cellID;
}
int CrossPoint::getPath1CellIndex(){
	return path1CellIndex;
}
int CrossPoint::getPath2CellIndex(){
	return path2CellIndex;
}

vector <CrossPoint> CrossPoint::findCrossPoints(Path* path1, Path* path2 ){

	vector <CrossPoint> crossoverPoints;
	CrossPoint crossoverPoint;

	int id=0;

	unsigned int c; //counter, will contain the index foe common cell in path2
	for (unsigned int i = 1; i < path1->getPath().size() -1; i++) {
		c = 1;
		while (c < path2->getPath().size() -1 && path1->getPath()[i] != path2->getPath()[c]) {
			c++;
		}
		if (c != path2->getPath().size() - 1) {
			crossoverPoint.cellID=id;
			crossoverPoint.path1CellIndex = i;
			crossoverPoint.path2CellIndex = c;
			id++;

			crossoverPoints.push_back(crossoverPoint);
		}
	}
	return crossoverPoints;
}

/****************************************************/
// Function: chooseRandomlyTwoCrossPoints
// Input: size of crossoverPoints vector
// Output: return randomly two cross points
/****************************************************/
vector <int> CrossPoint::chooseRandomlyTwoCrossPoints(uint size){

	vector <int> randomPoints;
	int randomCrossPoint1;
	int randomCrossPoint2;

	do{
	randomCrossPoint1 = rand() % size;
	randomCrossPoint2 = rand() % size;
	} while (randomCrossPoint1 == randomCrossPoint2);

	if (randomCrossPoint1>randomCrossPoint2){
		int temp=randomCrossPoint1;
		randomCrossPoint1=randomCrossPoint2;
		randomCrossPoint2=temp;
		}

	randomPoints.push_back(randomCrossPoint1);
	randomPoints.push_back(randomCrossPoint2);

	return randomPoints;
}

void CrossPoint::swapCrossPoints(){

	int temp = getPath1CellIndex();
	setPath1CellIndex(getPath2CellIndex());
	setPath2CellIndex(temp);
}


