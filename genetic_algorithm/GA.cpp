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

#include "GA.h"

using namespace std;

ofstream GA_file("GA_test.txt", ios::trunc);
ofstream GA_traceFile("GA_traceFile.txt", ios::trunc);
ofstream GA_bestGen("GA_bestGen.txt", ios::trunc);

GA::GA(void) {
}

GA::~GA(void) {
}

GA::GA(int numberOfIterations, uint populationS, int crossType, float crossProb, float mutationProb, int mutationIterationNumber, float minInitialPathCost, int radius_) {
	setNumberOfIterations(numberOfIterations);
	setPopulationSize(populationS);
	setCrossoverType(crossType);
	setCrossoverProbability(crossProb);
	setMutationProbability(mutationProb);
	setMutationIteration(mutationIterationNumber);
	setMinimumInitialPathCost(minInitialPathCost);
	setRadius(radius_);
}

GA::GA(int numberOfIterations, uint populationS, float crossProb,float mutationProb, int mutationIterationNumber, int radius_) {
	setNumberOfIterations(numberOfIterations);
	setPopulationSize(populationS);
	setCrossoverProbability(crossProb);
	setMutationProbability(mutationProb);
	setMutationIteration(mutationIterationNumber);
	setRadius(radius_);
}

void GA::setPopulationSize(uint populationS) {
	populationSize = populationS;
}

void GA::setCrossoverType(int crossType) {
	crossoverType = crossType;
}

void GA::setCrossoverProbability(float crossProb) {
	crossoverProbability = crossProb;
}

void GA::setMutationProbability(float mutationProb) {
	mutationProbability = mutationProb;
}

void GA::setMutationIteration(int mutationIterationNumber) {
	mutationIteration = mutationIterationNumber;
}

void GA::setMinimumInitialPathCost(float minInitialPathCost) {
	minimumInitialPathCost = minInitialPathCost;
}

void GA::setRadius(int radius_) {
	radius = radius_;
}

uint GA::getPopulationSize() {
	return populationSize;
}

int GA::getCrossoverType() {
	return crossoverType;
}

float GA::getCrossoverProbability() {
	return crossoverProbability;
}

float GA::getMutationProbability() {
	return mutationProbability;
}

int GA::getMutationIteration() {
	return mutationIteration;
}

float GA::getMinimumInitialPathCost() {
	return minimumInitialPathCost;
}

int GA::getRadius() {
	return radius;
}
/****************************************************/
// Function: getInitialPopulation
// Input: map, start and goal cells
// Output: a set of initial paths
// Description: it is used to generate a set of initial paths from start cell to goal cell
/****************************************************/
Population* GA::getInitialPopulation(OccupancyGridMap* OGM, int startCell, int goalCell) {

	Population* initialPopulation = new Population(); // contain a set of initial paths
	Population* emptyPopulation = new Population(); // to be returned in case of failure
	uint i = 1; // counter for population size

	/****************************************************/

	//Step 1. Got initial path between the start and the goal
	Path* initialPath = getInitialPath(OGM, startCell, goalCell, false);

	if (initialPath->getPath().size() < 3) {
		return emptyPopulation;
	}

	if (initialPath->getPathCost(OGM, true) > getMinimumInitialPathCost()) {
		i++;
		initialPopulation->insertPath(initialPath);
		initialPath->setName("Path #1");
		cout<<endl<<"generate initial path # 1,  cost = "<<initialPath->getPathCost(OGM, true)<<endl;;
	}

	do {

		//step 2: select random cell from the initial path
		uint index = getRandomCell(initialPath);

		int randomCell = initialPath->getPath()[index];
		//Step 3: find all free neighbors of the selected random cell with a radius of n cells
		vector<int> neighbors = getNeighborsAtRadiusN(OGM, getRadius(), randomCell);
		if (neighbors.size() != 0) {

			//Step 4: select one neighbor found in Step 3 and use it as cross point A
			int indexCrossPoint;

			do
				indexCrossPoint = rand() % (neighbors.size());
			while (neighbors[indexCrossPoint] == startCell|| neighbors[indexCrossPoint] == goalCell);

			int crossPoint = neighbors[indexCrossPoint];
			//Step 5: find a path between S and A

			Path* pathFromStartToCrossPoint = getInitialPath(OGM, startCell, crossPoint, true);
			if (pathFromStartToCrossPoint->getPath().size() > 0) {

				//Step 6: find a path between A and G
				Path* pathFromCrossPointToGoal = getInitialPath(OGM, crossPoint, goalCell, true);
				if (pathFromCrossPointToGoal->getPath().size() > 0) {

					//Step 7: construct the final Path
					Path* newPath = new Path();
					newPath->setPath(pathFromStartToCrossPoint->getPath());

					for (uint j = 1; j < pathFromCrossPointToGoal->getPath().size(); j++)
						newPath->insertCell(OGM, newPath->getPath().size(), pathFromCrossPointToGoal->getPath()[j]);

					if (newPath->getPathCost(OGM, true)> getMinimumInitialPathCost()) {

						// add newPath to the initialPopulation
						initialPopulation->insertPath(newPath);

						/****************************************************/
						stringstream ss;
						ss << i;
						string str = "Path #" + ss.str();
						newPath->setName(str);

						cout << endl << "generate initial path # " << i << ", cost = " << newPath->getPathCost(OGM, true) << endl;
						/****************************************************/
						i++;
					} else { } // do nothing if we want to skip that path
				}
			}
		} else {
			cout << "please choose a greater radius" << endl;
			radius++;
			// exit(1);
		}

	} while (i <= getPopulationSize());
	return initialPopulation;
}

/****************************************************/
// Function: evaluatePathFitness
// Input: map and path
// Output: fitness of the path
/****************************************************/

int GA::evaluatePathFitness(OccupancyGridMap* OGM, Path* path) {
	double fitness = 0;
	const int constant = 1;
	uint intFitness = 0;

	fitness = constant / path->getPathCost(OGM, true);
	intFitness = 100000000 * fitness; //convert fitness to int to avoid long float precision problem
	return intFitness;
}

/****************************************************/
// Function: rankSelection
//
// Input: map and population
// Output: return a selected set of paths (equal to the size of population)
/****************************************************/

Population* GA::rankSelection(OccupancyGridMap* OGM, Population* population) {

	vector<int> fitness;
	vector<int> pathIndices;
	double random;
	double probability;
	//vector <Path*> newPopulation;
	Population* newPop = new Population();
	// got the fitness for each path.
	for (uint i = 0; i < population->getPopulation().size(); i++)
		fitness.push_back(evaluatePathFitness(OGM, population->getPath(i)));

	//prepare vector of indices
	for (uint i = 0; i < population->getPopulation().size(); i++) {
		pathIndices.push_back(i);
	}

	// sort the paths according to their fitness
	sort(pathIndices, fitness);

	uint pathIndex = 0;
	do {
		if (pathIndex == population->getPopulation().size())
			pathIndex = 0;

		probability = ((float) (pathIndex + 1)
				/ (float) (population->getPopulation().size() + 1));
		random = ((double) rand() / RAND_MAX); // random number between 0 and 1

		if (random < probability)
			newPop->insertPath(population->getPath(pathIndices[pathIndex]));

		pathIndex++;

	} while (newPop->getPopulation().size() != getPopulationSize());

	return newPop;
}

/****************************************************/
// Function: sort [insertion sort]
// sort vector of paths according to their fitness
// Input: a map, set of path indices and their fitness
// Output:
/****************************************************/
void GA::sort(vector<int> &pathIndices, vector<int> &fitness) {

	int tempFitness;
	int tempPathIndex;
	int j;

	for (uint i = 1; i < fitness.size(); i++) {
		j = i;
		tempFitness = fitness[i];
		tempPathIndex = pathIndices[i];

		while (j > 0 && tempFitness < fitness[j - 1]) {

			fitness[j] = fitness[j - 1];
			pathIndices[j] = pathIndices[j - 1];
			j--;
		}

		fitness[j] = tempFitness;
		pathIndices[j] = tempPathIndex;
	}
}

/****************************************************/
// Function: crossover
// call appropriate crossover function based on crossoverType.
// Input: map two paths
// Output: return crossover result
/****************************************************/
vector<Path*> GA::crossover(OccupancyGridMap* OGM, Path* path1, Path* path2) {

	vector<Path*> offspringVector; // to store offspring paths from the crossover operation

	switch (getCrossoverType()) {
	case 1:
		offspringVector = onePointCrossover(OGM, path1, path2);
		break;
	case 2:
		offspringVector = twoPointCrossover(OGM, path1, path2);
		break;
	case 3:
		offspringVector = smartCrossover(OGM, path1, path2);
		break;
	default:
		// one point crossover is default case
		offspringVector = onePointCrossover(OGM, path1, path2);
		break;
	}

	return offspringVector;
}

/****************************************************/
// Function: twoPointcrossover
// choose randomly 2 cross points and perform two points crossover between them
// Input: two paths
// Output: return two offspring
/****************************************************/

vector<Path*> GA::twoPointCrossover(OccupancyGridMap* OGM, Path* path1, Path* path2) {

	Path* offspring = new Path(); // one offspring
	vector<Path*> offspringVector; // all offspring paths from the crossover operation
	vector<CrossPoint> crossPointVector; // all crossover points (common cells) between path1 and path2
	int randomCrossPoint1; // index of first selected cross point to perform crossover
	int randomCrossPoint2; // index of second selected cross point to perform crossover

	/******************************************************/
/*
	GA_file << "First path for crossover: " << endl << endl;
	for (uint i = 0; i < path1->getPath().size(); i++)
		GA_file << path1->getPath()[i] << "   ";
	GA_file << endl << endl;
	GA_file << "Cost = " << path1->getPathCost(OGM, true) << endl << endl;

	GA_file << "Second path for crossover: " << endl << endl;
	for (uint i = 0; i < path2->getPath().size(); i++)
		GA_file << path2->getPath()[i] << "   ";
	GA_file << endl << endl;
	GA_file << "Cost = " << path2->getPathCost(OGM, true) << endl << endl;
*/
	/******************************************************/
	//check the feasibility of the crossover
	if (!isCrossoverFeasible(OGM, path1, path2)) {
		return offspringVector; // return empty vector if the crossover not feasible.
	}
	/******************************************************/
	//find all crossover points (common cells) between path1 and path2
	crossPointVector = CrossPoint::findCrossPoints(path1, path2);

	// return empty vector if cross points are less than two.
	if (crossPointVector.size() < 2)
		return offspringVector;

	// choose randomly two cross points
	vector<int> crossPoints = CrossPoint::chooseRandomlyTwoCrossPoints(crossPointVector.size());

	/******************************************************/
	randomCrossPoint1 = crossPoints[0]; //i
	randomCrossPoint2 = crossPoints[1]; //j

	//check if two crossover points are in inverted positions
	bool invertCells = areCommonCellsInvert(crossPointVector[randomCrossPoint1], crossPointVector[randomCrossPoint2]);
	if (!invertCells) {
		/******************************************************/
/*
		GA_file << "First Cross Point index in path1 is " << crossPointVector[randomCrossPoint1].getPath1CellIndex()<< endl;
		GA_file << "Second Cross Point index in path1 is "<< crossPointVector[randomCrossPoint2].getPath1CellIndex()<< endl;
		GA_file << endl << endl;
		GA_file << "First Cross Point index in path2 is "<< crossPointVector[randomCrossPoint1].getPath2CellIndex()<< endl;
		GA_file << "Second Cross Point index in path2 is "<< crossPointVector[randomCrossPoint2].getPath2CellIndex()<< endl;
		GA_file << endl << endl;
*/
		/******************************************************/
		//generate first path: path1 - path2 - path1
		offspring = crossoverPath(OGM, path1, path2, crossPointVector[randomCrossPoint1], crossPointVector[randomCrossPoint2]);
		offspringVector.push_back(offspring);

		/******************************************************/
		offspring->setName("offspring1");
/*
		GA_file << offspring->getName() << ": ";
		for (uint i = 0; i < offspring->getPath().size(); i++)
			GA_file << offspring->getPath()[i] << "   ";
		GA_file << endl << endl;
		GA_file << "Cost = " << offspring->getPathCost(OGM, true) << endl<< endl;
*/
		/******************************************************/
		//generate second offspring: path2 - path1 - path2
		//first we need to swap cross point between path1 and path2, so we can consider path2 as path1
		crossPointVector[randomCrossPoint1].swapCrossPoints();
		crossPointVector[randomCrossPoint2].swapCrossPoints();
		offspring = crossoverPath(OGM, path2, path1, crossPointVector[randomCrossPoint1], crossPointVector[randomCrossPoint2]);
		offspringVector.push_back(offspring);

		/******************************************************/
		offspring->setName("offspring2");
/*
		GA_file << offspring->getName() << ": ";
		for (uint i = 0; i < offspring->getPath().size(); i++)
			GA_file << offspring->getPath()[i] << "   ";
		GA_file << endl << endl;
		GA_file << "Cost = " << offspring->getPathCost(OGM, true) << endl << endl;
		GA_file << "--------------" << endl << endl;
*/
		/******************************************************/
	}
	return offspringVector;
}

/****************************************************/
// Function: onePointCrossover
// choose randomly cross point and perform one point crossover on it
// Input: two paths
// Output: return two offspring
/****************************************************/
vector<Path*> GA::onePointCrossover(OccupancyGridMap* OGM, Path* path1, Path* path2) {

	Path* offspring = new Path(); // one offspring
	vector<Path*> offspringVector; // all offspring paths from the crossover operation
	vector<CrossPoint> crossPointVector; // all crossover points (common cells) between path1 and path2
	int randomCrossPoint; // index of cross point to perform crossover
	/******************************************************/
/*
	GA_file << "First path for crossover: " << endl << endl;
	for (uint i = 0; i < path1->getPath().size(); i++)
		GA_file << path1->getPath()[i] << "   ";
	GA_file << endl << endl;
	GA_file << "Cost = " << path1->getPathCost(OGM, true) << endl << endl;

	GA_file << "Second path for crossover: " << endl << endl;
	for (uint i = 0; i < path2->getPath().size(); i++)
		GA_file << path2->getPath()[i] << "   ";
	GA_file << endl << endl;
	GA_file << "Cost = " << path2->getPathCost(OGM, true) << endl << endl;
*/
	/******************************************************/
	//check crossover feasibility
	if (!isCrossoverFeasible(OGM, path1, path2))
		return offspringVector; // return empty vector if the crossover not feasible.

	/******************************************************/
	//find all crossover points (common cells) between path1 and path2
	crossPointVector = CrossPoint::findCrossPoints(path1, path2);

	// return empty vector if there is no cross points.
	if (crossPointVector.size() < 1)
		return offspringVector;

	//choose randomly cross point to perform crossover
	randomCrossPoint = rand() % crossPointVector.size();

	/******************************************************/
/*
	GA_file << "Cross Point index in path1 is "<< crossPointVector[randomCrossPoint].getPath1CellIndex() << endl;
	GA_file << endl << endl;
	GA_file << "First Cross Point index in path2 is " << crossPointVector[randomCrossPoint].getPath2CellIndex() << endl;
	GA_file << endl << endl;
*/
	/******************************************************/
	//generate first offspring
	offspring = crossoverPath(OGM, path1, path2, crossPointVector[randomCrossPoint]);
	offspringVector.push_back(offspring);

	/******************************************************/
/*
	offspring->setName("offspring1");
	GA_file << offspring->getName() << ": ";
	for (uint i = 0; i < offspring->getPath().size(); i++)
		GA_file << offspring->getPath()[i] << "   ";
	GA_file << endl << endl;
	GA_file << "Cost = " << offspring->getPathCost(OGM, true) << endl << endl;
*/
	/******************************************************/
	//generate second offspring
	//first we need to swap cross point between path1 and path2, so we can consider path2 as path1
	crossPointVector[randomCrossPoint].swapCrossPoints();

	offspring = crossoverPath(OGM, path2, path1, crossPointVector[randomCrossPoint]);
	offspringVector.push_back(offspring);

	/******************************************************/
	offspring->setName("offspring2");
/*
	GA_file << offspring->getName() << ": ";
	for (uint i = 0; i < offspring->getPath().size(); i++)
		GA_file << offspring->getPath()[i] << "   ";
	GA_file << endl << endl;
	GA_file << "Cost = " << offspring->getPathCost(OGM, true) << endl << endl;
	GA_file << "--------------" << endl << endl;
*/
	/******************************************************/
	return offspringVector;
}

/****************************************************/
// Function: smartCrossover
// smartCrossover will choose the shortest part from each path
// Input: map and two paths
// Output: one offspring
/****************************************************/

vector<Path*> GA::smartCrossover(OccupancyGridMap* OGM, Path* path1, Path* path2) {

	vector<Path*> offspringVector; // to store offspring paths from the crossover operation;
	Path* offspring = new Path();
	vector<CrossPoint> crossPointVector;
	vector<int> crossPoints; //indices of 2 cross points to perform the crossover

	float distPath1BeforePoint1; //distance of path1 from start cell to first crossover point
	float distPath1Point1Point2; //distance of path1 from first crossover point to second crossover point
	float distPath1AfterPoint2; //distance of path1 from second crossover point to the goal cell

	float distPath2BeforePoint1; //distance of path2 from start cell to first crossover point
	float distPath2Point1Point2; //distance of path2 from first crossover point to second crossover point
	float distPath2AfterPoint2; //distance of path2 from second crossover point to the goal cell

	/******************************************************/
/*
	GA_file << "First path for crossover: " << endl << endl;
	for (uint i = 0; i < path1->getPath().size(); i++)
		GA_file << path1->getPath()[i] << "   ";
	GA_file << endl << endl;
	GA_file << "Cost = " << path1->getPathCost(OGM, true) << endl << endl;

	GA_file << "Second path for crossover: " << endl << endl;
	for (uint i = 0; i < path2->getPath().size(); i++)
		GA_file << path2->getPath()[i] << "   ";
	GA_file << endl << endl;
	GA_file << "Cost = " << path2->getPathCost(OGM, true) << endl << endl;
*/
	/******************************************************/
	//check crossover feasibility
	if (!isCrossoverFeasible(OGM, path1, path2))
		return offspringVector; // return empty path if the crossover not feasible.

	/******************************************************/
	//find all crossover points (common cells)
	crossPointVector = CrossPoint::findCrossPoints(path1, path2);

	// if common cells less than two, will return empty vector
	if (crossPointVector.size() < 2)
		return offspringVector;

	// choose randomly two cross points
	crossPoints = CrossPoint::chooseRandomlyTwoCrossPoints(crossPointVector.size());

	int randomCrossPoint1 = crossPoints[0]; //i
	int randomCrossPoint2 = crossPoints[1]; //j

	//check if two crossover points are in inverted positions
	bool invertCells = areCommonCellsInvert(crossPointVector[randomCrossPoint1],
			crossPointVector[randomCrossPoint2]);
	/******************************************************/

	if (!invertCells) {

		/******************************************************/
/*
		GA_file << "First Cross Point index in path1 is " << crossPointVector[randomCrossPoint1].getPath1CellIndex()<< endl;
		GA_file << "Second Cross Point index in path1 is "<< crossPointVector[randomCrossPoint2].getPath1CellIndex()<< endl;
		GA_file << endl << endl;
		GA_file << "First Cross Point index in path2 is "<< crossPointVector[randomCrossPoint1].getPath2CellIndex()<< endl;
		GA_file << "Second Cross Point index in path2 is "<< crossPointVector[randomCrossPoint2].getPath2CellIndex()<< endl;
		GA_file << endl << endl;
*/
		/******************************************************/
		// distance of first path
		distPath1BeforePoint1 = path1->calculateDistance(OGM, 0, crossPointVector[randomCrossPoint1].getPath1CellIndex());
		distPath1Point1Point2 = path1->calculateDistance(OGM, crossPointVector[randomCrossPoint1].getPath1CellIndex(), crossPointVector[randomCrossPoint2].getPath1CellIndex());
		distPath1AfterPoint2 = path1->calculateDistance(OGM, crossPointVector[randomCrossPoint2].getPath1CellIndex(), path1->getPath().size() - 1);

		// distance of second path
		distPath2BeforePoint1 = path2->calculateDistance(OGM, 0, crossPointVector[randomCrossPoint1].getPath2CellIndex());
		distPath2Point1Point2 = path2->calculateDistance(OGM, crossPointVector[randomCrossPoint1].getPath2CellIndex(), crossPointVector[randomCrossPoint2].getPath2CellIndex());
		distPath2AfterPoint2 = path2->calculateDistance(OGM, crossPointVector[randomCrossPoint2].getPath2CellIndex(), path2->getPath().size() - 1);

		/******************************************************/
		// choose first part of the offspring
		if (distPath1BeforePoint1 < distPath2BeforePoint1) {
			offspring = crossoverPath(OGM, offspring, path1, 0, crossPointVector[randomCrossPoint1].getPath1CellIndex());
		} else
			offspring = crossoverPath(OGM, offspring, path2, 0, crossPointVector[randomCrossPoint1].getPath2CellIndex());

		/******************************************************/
		// choose second part of the offspring
		if (distPath1Point1Point2 < distPath2Point1Point2) {
			offspring = crossoverPath(OGM, offspring, path1,crossPointVector[randomCrossPoint1].getPath1CellIndex(), crossPointVector[randomCrossPoint2].getPath1CellIndex());
		} else
			offspring = crossoverPath(OGM, offspring, path2, crossPointVector[randomCrossPoint1].getPath2CellIndex(), crossPointVector[randomCrossPoint2].getPath2CellIndex());

		/******************************************************/
		// choose third part of the offspring
		if (distPath1AfterPoint2 < distPath2AfterPoint2) {
			offspring = crossoverPath(OGM, offspring, path1, crossPointVector[randomCrossPoint2].getPath1CellIndex(), path1->getPath().size());
		} else
			offspring = crossoverPath(OGM, offspring, path2, crossPointVector[randomCrossPoint2].getPath2CellIndex(), path2->getPath().size());

		/******************************************************/
		offspringVector.push_back(offspring);

		/******************************************************/
		offspring->setName("offspring");
/*
		GA_file << offspring->getName() << ": ";
		for (uint i = 0; i < offspring->getPath().size(); i++)
			GA_file << offspring->getPath()[i] << "   ";
		GA_file << endl << endl;
		GA_file << "Cost = " << offspring->getPathCost(OGM, true) << endl << endl;
*/
	}

	return offspringVector;
}

/****************************************************/
// Function: crossoverPath [for two points crossover]
// Input: map, two paths and two cells
// Output: return the path generated by the crossover operator
/****************************************************/
Path* GA::crossoverPath(OccupancyGridMap* OGM, Path* path1, Path* path2, CrossPoint point1, CrossPoint point2) {

	Path* path = new Path();
	int index = 0;

	/*****************perform the crossover******************/

	// Add the first part from first path
	for (int i = 0; i < point1.getPath1CellIndex(); i++) {
		path->insertCell(OGM, index, path1->getPath()[i]);
		index++;
	}
	// add second part of the second path
	for (int i = point1.getPath2CellIndex(); i < point2.getPath2CellIndex(); i++) {
		path->insertCell(OGM, index, path2->getPath()[i]);
		index++;
	}
	// add third part of the first path
	for (uint i = point2.getPath1CellIndex(); i < path1->getPath().size(); i++) {
		path->insertCell(OGM, index, path1->getPath()[i]);
		index++;
	}
	return path;
}

/****************************************************/
// Function: crossoverPath [for one point crossover]
// Input: map, two paths and one cell
// Output: return the path generated by the crossover operator
/****************************************************/
Path* GA::crossoverPath(OccupancyGridMap* OGM, Path* path1, Path* path2, CrossPoint point) {

	Path* path = new Path();
	int index = 0;

	/*****************perform the crossover******************/

	// Add the first part from first path
	for (int i = 0; i < point.getPath1CellIndex(); i++) {
		path->insertCell(OGM, index, path1->getPath()[i]);
		index++;
	}
	// add second part of the second path
	for (uint i = point.getPath2CellIndex(); i < path2->getPath().size(); i++) {
		path->insertCell(OGM, index, path2->getPath()[i]);
		index++;
	}
	return path;
}

/****************************************************/
// Function: crossoverPath [for smart crossover]
// Add to a smartPath appropriate part from path1
// Input: map, two paths and two cell indices
// Output: return the path generated by the crossover operator
/****************************************************/
Path* GA::crossoverPath(OccupancyGridMap* OGM, Path* smartPath, Path* path1, int cell1, int cell2) {

	for (int i = cell1; i < cell2; i++)
		smartPath->insertCell(OGM, smartPath->getPath().size(), path1->getPath()[i]);

	return smartPath;
}

/****************************************************/
// Function: isCrossoverFeasible
// To check if the crossover can be performed on the two paths.
// The crossover is feasible if:
// 1- the paths are feasible.
// 2- the paths are different
// 3- the paths have same start and goal
// Input: map and two paths
// Output: return true if the crossover feasible, false otherwise
/****************************************************/

inline bool GA::isCrossoverFeasible(OccupancyGridMap* OGM, Path* path1, Path* path2) {

	if (!path1->isFeasible(OGM) || !path2->isFeasible(OGM)
	|| areTwoPathsEqual(OGM, path1, path2)
	|| path1->getPath()[0] != path2->getPath()[0]
	|| path1->getPath()[path1->getPath().size() - 1] != path2->getPath()[path2->getPath().size() - 1])
		return false;

	return true;
}

/****************************************************/
// Function: areCommonCellsInvert
// Check if the two common cells in inverted position
// Input: Indices of the common cells
// Output: return true if they inverted, false otherwise
/****************************************************/

inline bool GA::areCommonCellsInvert(CrossPoint Point1, CrossPoint Point2) {

	if (Point1.getPath1CellIndex() < Point2.getPath1CellIndex() && Point1.getPath2CellIndex() > Point2.getPath2CellIndex())
		return true;

	return false;
}

/****************************************************/
// Function: mutatePath
// Input: Map and path
// Output: path after mutation
/****************************************************/
Path* GA::mutatePath(OccupancyGridMap* OGM, Path* path) {

	Path* mutatedPath = new Path();
	Path* feasibleMutatedPath = new Path();
	int round = 0;

	/****************************************************/
	// if the path not feasible, return empty path
	if (!path->isFeasible(OGM))
		return feasibleMutatedPath;

	/****************************************************/
/*
	GA_file << "The selected path for mutation: " << endl << endl;
	GA_file << path->getName() << ": ";
	for (uint i = 0; i < path->getPath().size(); i++)
		GA_file << path->getPath()[i] << "   ";
	GA_file << endl << endl;
	GA_file << "Cost = " << path->getPathCost(OGM, true) << endl << endl;
*/
	/****************************************************/
	mutatedPath->setPath(path->getPath());

	do {
		for (uint i = 0; i < path->getPath().size(); i++)
			mutatedPath->setCell(OGM, path->getPath()[i], i);

		mutatedPath = mutateCell(OGM, mutatedPath);
		round++;

	} while (round < getMutationIteration() && !mutatedPath->isFeasible(OGM));

	if (mutatedPath->isFeasible(OGM)) {
		feasibleMutatedPath->setPath(mutatedPath->getPath());
		feasibleMutatedPath->setName("Mutated path");
	} else { // If the mutated path not feasible

		if (path->getPath().size() > 3) { //TODO check
			// got the original path
			for (uint i = 0; i < path->getPath().size(); i++)
				mutatedPath->setCell(OGM, path->getPath()[i], i);
			//mutate sub path
			feasibleMutatedPath = mutateSubPath(OGM, mutatedPath);
			feasibleMutatedPath->setName("Mutated path");
		} else { // in case fail of mutation return the original path
			feasibleMutatedPath->setPath(path->getPath());

		}
	}

	/****************************************************/
/*
	GA_file << feasibleMutatedPath->getName() << ": ";
	for (uint i = 0; i < feasibleMutatedPath->getPath().size(); i++)
		GA_file << feasibleMutatedPath->getPath()[i] << "   ";
	GA_file << endl << endl << "Cost = " << feasibleMutatedPath->getPathCost(OGM, true) << endl;
	GA_file << endl << "----------" << endl;
*/
	/****************************************************/

	return feasibleMutatedPath;
}

/****************************************************/
// Function: mutateCell
// Input: Map and path
// Output: path after mutate one cell
/****************************************************/
inline Path* GA::mutateCell(OccupancyGridMap* OGM, Path* path) {

	uint randomCell;
	vector<int> neighbors;
	int randomNeighbor;
	Path* mutatedPath = new Path();
	mutatedPath->setPath(path->getPath());

	// choose random cell
	randomCell = getRandomCell(path);

	// find all free neighbors for randomCell
	neighbors = findFreeNeighborCell(OGM, path->getPath()[randomCell]);

	//choose random neighbor
	randomNeighbor = rand() % neighbors.size();

	//replace the selected cell with its neighbor
	mutatedPath->setCell(OGM, neighbors[randomNeighbor], randomCell);

	/****************************************************/
	if (mutatedPath->isFeasible(OGM)) {
/*
		GA_file << "Cell selected for mutation is: " << path->getPath()[randomCell] << endl << endl;
		GA_file << "Cell after mutation is: " << mutatedPath->getPath()[randomCell] << endl << endl;
*/
	}

	/****************************************************/
	return mutatedPath;
}

/****************************************************/
// Function: mutateSubPath
// Input: Map and path
// Output: path after mutate sub part of it
/****************************************************/
Path* GA::mutateSubPath(OccupancyGridMap* OGM, Path* path) {

	uint randomCell1;
	uint randomCell2;
	int differenceBetweenRandomCells;

	vector<int> neighbors;
	int randomNeighbor;

	Path* mutatedPath = new Path();
	Path* subPath = new Path();
	int round = 0;

	/****************************************************/
	//mutatedPath->setPath(path->getPath());
	do {
		randomCell1 = getRandomCell(path);
		randomCell2 = getRandomCell(path);

		// swap the random cells if the first greater than second
		if (randomCell1 > randomCell2) {

			uint temp = randomCell1;
			randomCell1 = randomCell2;
			randomCell2 = temp;
		}
		differenceBetweenRandomCells = randomCell1 - randomCell2;

		// find all free neighbors for randomCell1
		neighbors = findFreeNeighborCell(OGM, path->getPath()[randomCell1]);
		round++;
	} while ((neighbors.size() < 4 || !(differenceBetweenRandomCells < -2)) && round < getMutationIteration());
	/****************************************************/
	if (neighbors.size() < 4 || !(differenceBetweenRandomCells < -2)){ //mutation fails, return original path

		return path;
	}

	//choose random neighbor of the selected cell,
	//and make sure it is not same as current neighbors of the cell in path1 or same as the random neighbor 2.
	do {
		randomNeighbor = rand() % neighbors.size();
	} while (neighbors[randomNeighbor] == path->getPath()[randomCell1 + 1]
			|| neighbors[randomNeighbor] == path->getPath()[randomCell1 - 1]
			|| neighbors[randomNeighbor] == path->getPath()[randomCell2]);

	/****************************************************/
	//generate new sub path from a selected neighbor of randomCell1 to randomCell2
	subPath = findSubPath(OGM, neighbors[randomNeighbor], path->getPath()[randomCell2]);


	//TODO consider the case when subPath is empty for any reason so the mutation failed
	if (! subPath->getPath().size()>0)
			return path;
	/*
	 // Remove old subPath
	 for(int i=randomCell2; i>=randomCell1+1; i--)
	 mutatedPath->removeCell(OGM, i);

	 //add new subPath
	 for(uint i=0; i<subPath->getPath().size(); i++)
	 mutatedPath->insertCell(OGM, i+randomCell1+1, subPath->getPath()[i]);
	 */

	//construct new mutated path.
	// add first part from original path until random cell1
	for (uint i = 0; i <= randomCell1; i++)
		mutatedPath->insertCell(OGM, i, path->getPath()[i]);

	//add new subPath
	for (uint i = 0; i < subPath->getPath().size(); i++)
		mutatedPath->insertCell(OGM, i + randomCell1 + 1, subPath->getPath()[i]);

	//add last part from original path from randomCell2 +1 until end of the path
	for (uint i = randomCell2 + 1; i < path->getPath().size(); i++)
		mutatedPath->insertCell(OGM, mutatedPath->getPath().size(), path->getPath()[i]);

	/****************************************************/
/*
	GA_file << "Sub path selected for mutation is: [" << path->getPath()[randomCell1] << "]";
	for (int i = randomCell1 + 1; i < randomCell2; i++) {
		GA_file << path->getPath()[i] << " ";
	}
	GA_file << "[" << path->getPath()[randomCell2] << "]" << endl << endl;
	GA_file << "cost of the old subpath = " << path->calculateDistance(OGM, randomCell1, randomCell2) << endl << endl;

	GA_file << "Sub path after the mutation is:  [" << mutatedPath->getPath()[randomCell1] << "]";

	int i = randomCell1 + 1;
	do {
		GA_file << mutatedPath->getPath()[i] << " ";
		i++;
	} while (mutatedPath->getPath()[i] != path->getPath()[randomCell2]);
	GA_file << "[" << mutatedPath->getPath()[i] << "]" << endl << endl;
	GA_file << "cost of the new subpath = " << mutatedPath->calculateDistance(OGM, randomCell1, i);
	GA_file << endl << endl;
*/
	/****************************************************/

	return mutatedPath;
}


/****************************************************/
// Function: findSubPath
// Input: map and 2 cells
// Output: path between 2 cells
/****************************************************/
Path* GA::findSubPath(OccupancyGridMap* OGM, int cell1, int cell2) {

	Path* subPath = new Path();

	subPath = getInitialPath(OGM, cell1, cell2, false);

	return subPath;
}

/****************************************************/
// Function: getRandomCell
// Input: path
// Output: return one cell randomly
/****************************************************/
inline uint GA::getRandomCell(Path* path) {
	uint min = 1;

	uint max = path->getPath().size() - 1;

	uint randomCell = (rand() % (max - min)) + min;

	return randomCell;
}

/****************************************************/
// Function: findPath
// Input: map and initial population
// Output: best path found by GA
/****************************************************/
Path* GA::findPath(OccupancyGridMap* OGM, Population* pop,timespec & bestExecutionTime) {

	//vector <Path*> currentPopulation;
	Population* currentPop = new Population();
	Population* bestPaths = new Population();
	vector <timespec> bestExecutionTimes;
	timespec time1,time2;
	int crossoverCount = 0;

	//currentPopulation=initialPopulation;
	currentPop = pop;

	/****************************************************/
/*
	GA_file << "Number of iterations = " << getNumberOfIterations() << endl;
	GA_file << "population size = " << getPopulationSize() << endl;
	GA_file << "Crossover type =" << getCrossoverType() << endl;
	GA_file << "Crossover probability = " << getCrossoverProbability() << endl;
	GA_file << "Mutation probability = " << getMutationProbability() << endl;
	GA_file << "Mutation iteration number = " << getMutationIteration() << endl;

	GA_file << endl << "*****************Initial Population*****************" << endl << endl;

	for (unsigned int j = 0; j < pop->getPopulation().size(); j++) {
		GA_file << pop->getPath(j)->getName() << " : ";
		for (unsigned int i = 0; i < pop->getPath(j)->getPath().size(); i++)
			GA_file << pop->getPath(j)->getPath()[i] << " ";
		GA_file << endl << endl << "Cost = " << pop->getPath(j)->getPathCost(OGM, true)
				<< ", Fitness = " << evaluatePathFitness(OGM, pop->getPath(j)) << endl << endl;
	}
	GA_file << endl << "-------------------------------------------------------------------" << endl << endl;
*/
	/****************************************************/
	int NIGA = 0;
	double random;
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);
	for (NIGA = 0; NIGA < getNumberOfIterations(); NIGA++) { //repeat GA NumberOfIterations times

		Population* currentPopulationAfterSelection = new Population();

		Population* nextGen = new Population();
		int bestPathIndex;

		//GA_file << endl << "*****************Iteration # " << NIGA + 1 << "*****************" << endl << endl;
		cout << endl << "**********Iteration # " << NIGA + 1 << "**********" << endl;

		// Add best path in the current population immediately into the next generation
		bestPathIndex = findBestPath(OGM, currentPop);
		nextGen->insertPath(currentPop->getPath(bestPathIndex));

		if ( getCrossoverType()== 1  && currentPop->getPath(bestPathIndex)->getPath().size() < 4 ){
			cout<<endl<<"The GA aborted because the best path can not be enhanced"<<endl;

			//if this is the first iteration
			if (NIGA == 0){
				return currentPop->getPath(bestPathIndex);
			}
			else{
				// find best path generation
				Path* bestPath_ = new Path();
				int bestPathIndex_ = findBestPath(OGM, bestPaths);
				bestPath_->setPath(bestPaths->getPath(bestPathIndex_)->getPath());
				cout << endl << "best path found in generation # "<< bestPathIndex_ + 1 << endl;
				//GA_bestGen << bestPathIndex_ + 1 <<" aborted in iteration "<<NIGA+1<< endl; // trace
				return currentPop->getPath(bestPathIndex);
			}
		}

		if ( (getCrossoverType()== 2 || getCrossoverType()== 3) && currentPop->getPath(bestPathIndex)->getPath().size() < 5 ){
			cout<<endl<<"The GA aborted because the best path can not be enhanced"<<endl;
			//if this is the first iteration
				if (NIGA == 0){
					return currentPop->getPath(bestPathIndex);
				}
				else{
					// find best path generation
					Path* bestPath_ = new Path();
					int bestPathIndex_ = findBestPath(OGM, bestPaths);
					bestPath_->setPath(bestPaths->getPath(bestPathIndex_)->getPath());
					cout << endl << "best path found in generation # "<< bestPathIndex_ + 1 << endl;
					//GA_bestGen << bestPathIndex_ + 1 <<" aborted in iteration "<<NIGA+1<< endl; // trace
					return currentPop->getPath(bestPathIndex);
				}
		}

		/****************************************************/
		cout << endl << "--------Selection--------" << endl;
		//GA_file << "Step1: Selection --------" << endl << endl;

		//Select solutions for next population
		currentPopulationAfterSelection = rankSelection(OGM, currentPop);

		// print population after the selection
/*
		for (unsigned int j = 0; j < currentPopulationAfterSelection->getPopulation().size(); j++) {
			GA_file << currentPopulationAfterSelection->getPath(j)->getName() << " : ";
			for (unsigned int i = 0; i < currentPopulationAfterSelection->getPath(j)->getPath().size(); i++)
				GA_file << currentPopulationAfterSelection->getPath(j)->getPath()[i] << " ";
			GA_file << endl << endl << "Cost = " << currentPopulationAfterSelection->getPath(j)->getPathCost(OGM, true)
					<< ", Fitness = " << evaluatePathFitness(OGM, currentPopulationAfterSelection->getPath(j)) << endl << endl;
		}
		GA_file << endl << "-------------------------------------------------------------------" << endl << endl;
*/
		/****************************************************/
		//Perform crossover on current population to generate the next generation
		cout << endl << "--------Crossover--------" << endl;
		//GA_file << "Step2: Crossover --------" << endl << endl;

		do {
			int randomPathIndex1;
			int randomPathIndex2;
			vector<Path*> crossoverOffspring;

			// choose randomly two path from the current population
			do {
				randomPathIndex1 = rand() % currentPopulationAfterSelection->getPopulation().size();
				randomPathIndex2 = rand() % currentPopulationAfterSelection->getPopulation().size();

			} while (randomPathIndex1 == randomPathIndex2);

			//GA_file << "Indices of paths selected for crossover, path#1 index = " << randomPathIndex1 << ", path#2 index = " << randomPathIndex2 << endl << endl;

			random = ((double) rand() / RAND_MAX); // random number between 0 and 1 for crossover probability

			if (random < crossoverProbability) {
				// perform crossover between paths
				crossoverOffspring = crossover(OGM, currentPopulationAfterSelection->getPath(randomPathIndex1), currentPopulationAfterSelection->getPath(randomPathIndex2));

				if (crossoverOffspring.size() > 0) {
					for (uint j = 0; j < crossoverOffspring.size(); j++)
						nextGen->insertPath(crossoverOffspring[j]);
					crossoverCount=0;
				} else {
					//No offspring, try another paths
					crossoverCount++;
					//GA_file << endl << "Crossover is not possible, choose other paths." << endl;
					//GA_file << endl << "-------------" << endl << endl;
				}
			} else { // In case the crossover will not performed, add the parent to next generation without change.
					 // only when the probability less than 1
				nextGen->insertPath(currentPopulationAfterSelection->getPath(randomPathIndex1));
				nextGen->insertPath(currentPopulationAfterSelection->getPath(randomPathIndex2));
			}

		} while (nextGen->getPopulation().size() < populationSize && crossoverCount<50);

		if ( crossoverCount >= 50 ){
					cout<<endl<<"The GA aborted because the best path can not be enhanced "<<endl;
					//if this is the first iteration
					if (NIGA == 0){
						return currentPop->getPath(bestPathIndex);
					}
					else{
						// find best path generation
						Path* bestPath_ = new Path();
						int bestPathIndex_ = findBestPath(OGM, bestPaths);
						bestPath_->setPath(bestPaths->getPath(bestPathIndex_)->getPath());
						cout << endl << "best path found in generation # "<< bestPathIndex_ + 1 << endl;
						//GA_bestGen << bestPathIndex_ + 1 <<" aborted in iteration "<<NIGA+1 << endl; // trace
						return currentPop->getPath(bestPathIndex);
					}
				}

		//GA_file << endl << "-------------------------------------------------------------------" << endl << endl;

		/****************************************************/
		//mutation
		//GA_file << "Step3: Mutation --------" << endl << endl;
		cout << endl << "--------Mutation---------" << endl;

		for (uint i = 1; i < nextGen->getPopulation().size(); i++) {
			random = ((double) rand() / RAND_MAX); // random number between 0 and 1
			if (random < mutationProbability) {

				Path* mutatedPath = new Path();
				mutatedPath = mutatePath(OGM, nextGen->getPath(i));
				if (mutatedPath->getPath().size() > 0)
					nextGen->setPath(i, mutatedPath);
			}
		}

		// TODO check the mutation probability for each cell

		//GA_file << endl << "-------------------------------------------------------------------" << endl << endl;

		/****************************************************/
		// Fix the names of the paths of next generation
		for (uint i = 0; i < nextGen->getPopulation().size(); i++) {
			stringstream pathNum;
			pathNum << i;
			string pathName = "Path #" + pathNum.str();
			nextGen->getPath(i)->setName(pathName);
		}
		/****************************************************/
		//cout << endl << "*****************Generation #" << NIGA + 1 << "*****************" << endl << endl;
		//GA_file << endl << "*****************Generation #" << NIGA + 1 << "*****************" << endl << endl;
/*
		for (unsigned int j = 0; j < nextGen->getPopulation().size(); j++) {
			GA_file << nextGen->getPath(j)->getName() << " : ";
			for (unsigned int i = 0; i < nextGen->getPath(j)->getPath().size(); i++)
				GA_file << nextGen->getPath(j)->getPath()[i] << " ";
			GA_file << endl << endl << "Cost = " << nextGen->getPath(j)->getPathCost(OGM, true)
					<< ", Fitness = " << evaluatePathFitness(OGM, nextGen->getPath(j)) << endl << endl;
		}

		GA_file << endl << "-------------------------------------------------------------------"<< endl << endl;
*/
		/****************************************************/
		currentPop = nextGen;

		// find best path in this iteration
		bestPathIndex = findBestPath(OGM, nextGen);
		cout <<endl<< "best path cost in this generation = " << nextGen->getPath(bestPathIndex)->getPathCost(OGM, true) << endl;
/*
		GA_file << endl << "*******Best path in this generation*******" << endl<< endl;
		GA_file << nextGen->getPath(bestPathIndex)->getName() << " : ";
		for (unsigned int i = 0; i < nextGen->getPath(bestPathIndex)->getPath().size(); i++)
			GA_file << nextGen->getPath(bestPathIndex)->getPath()[i] << " ";
		GA_file << endl << endl << "Cost = " << nextGen->getPath(bestPathIndex)->getPathCost(OGM, true)
				<< ", Fitness = " << evaluatePathFitness(OGM, nextGen->getPath(bestPathIndex)) << endl << endl;
		GA_file << endl << "-------------------------------------------------------------------" << endl << endl;
*/
		bestPaths->insertPath(nextGen->getPath(bestPathIndex));
		clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
		timespec executionTime=diff(time1,time2); 
		bestExecutionTimes.push_back(executionTime);

		//cout << endl << "Time of this generation: " << iterationTime << endl;

		/*************************Trace file***************************/
		/*
		GA_traceFile << "Generation #" << NIGA + 1 << ",  ";
		for (unsigned int i = 0; i < nextGen->getPath(bestPathIndex)->getPath().size(); i++)
			GA_traceFile << nextGen->getPath(bestPathIndex)->getPath()[i] << " ";
		GA_traceFile << ", time: " << GATime << endl << endl;
*/
		/****************************************************/
	}
	/****************************************************/
	//GA_file << endl << endl << "************END of GA *************" << endl << endl;

	// find best path
	Path* bestPath = new Path();
	int bestPathIndex = findBestPath(OGM, bestPaths);
	bestPath->setPath(bestPaths->getPath(bestPathIndex)->getPath());
        bestExecutionTime=bestExecutionTimes[bestPathIndex] ;

	cout << endl << "best path found in generation # "<< bestPathIndex + 1;
	cout << endl << endl << "************END of GA*************" << endl << endl;

	//GA_bestGen << bestPathIndex + 1 << endl; // trace
/*
	GA_file << "Best path found is" << endl;
	for (unsigned int i = 0; i < bestPath->getPath().size(); i++)
		GA_file << bestPath->getPath()[i] << " ";
	GA_file << endl << endl << "Cost = " << bestPath->getPathCost(OGM, true)
			<< ", Fitness = " << evaluatePathFitness(OGM, bestPath) << endl<< endl;
	GA_file << endl << "-------------------------------------------------------------------" << endl << endl;
*/
	/****************************************************/

	cout << endl;

	return bestPath;
}

/****************************************************/
// Function: findBestPath
// Input: map and vector of paths
// Output: index of shortest path
/****************************************************/
int GA::findBestPath(OccupancyGridMap* OGM, Population* pop) {

	int indexBestPath = 0;
	//float minPathCost=paths[0]->getPathCost(OGM,true);
	float minPathCost = pop->getPath(0)->getPathCost(OGM, true);

	for (uint i = 0; i < pop->getPopulation().size(); i++) {
		if (pop->getPath(i)->getPathCost(OGM, true) < minPathCost) {
			minPathCost = pop->getPath(i)->getPathCost(OGM, true);
			indexBestPath = i;
		}
	}
	return indexBestPath;
}


int clock_gettime(clockid_t clk_id, struct timespec *tp);
timespec GA::diff(timespec start, timespec end)
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
