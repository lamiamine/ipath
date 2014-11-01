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

#include "Population.h"

Population::Population(void) {
}

Population::~Population(void) {
}

void Population::setPopulation(vector <Path*> populationPaths){

	  for(uint i=0; i<populationPaths.size(); i++){
		  population.push_back(populationPaths[i]);
	  }
}

void Population::setPopulationSize(uint popSize){
	populationSize = popSize;
}


vector <Path*> Population::getPopulation(){
	return population;
}

uint Population::getPopulationSize(){
	return populationSize;
}

void Population::setPath(int pathIndex, Path* path){
	population[pathIndex]=path;
}

Path* Population::getPath(int pathIndex){
	Path* path = new Path();
	path->setPath(population[pathIndex]->getPath());
	return path;
}

void Population::insertPath( Path* path){
		  population.push_back(path);
}

void Population::printPopulation(){

    for (unsigned int j=0; j < population.size();j++){
    	cout<<endl<<endl;
    	cout<<population[j]->getName()<<" : ";
    	for(unsigned int i=0; i<population[j]->getPath().size();i++)
    		cout<<population[j]->getPath()[i]<<" ";
    	cout<<endl<<endl;
    }
}
