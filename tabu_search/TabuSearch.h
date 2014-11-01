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


#ifndef TABUSEARCH_H
#define TABUSEARCH_H

#include "tabu_move/TabuMove.h"
#include<vector>
#include <list>
#include "../generic_path_planner/GenericPathPlanner.h"

	/**
	* @struct TabuSearch
	* @brief A class that implements the tabu search
	*/
class TabuSearch: public GenericPathPlanner{
  public:
	/**
	* @struct Move
	* @brief 
	*/
    struct Move {
    Path* newPathAfterMove; // new Path generated after a move
    int previousCell; // we will need it for swap and insert
    int position; // we will need it in all cases (insert+remove+swap)
    string moveType; //insert, remove or swap
    bool aspirationCriteria;
    };
	/**
	* @brief Constructor
	*/
    TabuSearch(void);
	/**
	* @brief Constructor
	* @param size
	* @param t
	* @param numberOfIterations
	*/
    TabuSearch(int size, int t, int numberOfIterations);
	/**
	* @brief Destructor.
	*/
    ~TabuSearch(void); 
    
    void setBestPath(Path* path);
    void setTabuListIn(list<TabuMove> TLIN);
    void setTabuListOut(list<TabuMove> TLOUT);
    void setTabuListSize(int size);
    void setTenure(int t);
    
    Path* getBestPath();
    list<TabuMove> getTabuListIn();
    list<TabuMove> getTabuListOut();
    int getTabuListSize();
    int getTenure();
    
	/**
	* @brief it is used to calculate the execution time
	* @param start
	* @param end
	* @return execution time
	*/
     timespec diff(timespec start, timespec end);
    //int clock_gettime(clockid_t clk_id, struct timespect *tp);
 	/**
 	* @brief it is used to verify if a move is tabu or not
 	* @param cellID1
 	* @param cellID2
 	* @param TabuList
 	* @return true if a move is tabu, false otherwise
 	*/
    bool isTabu(int cellID1, int cellID2, list<TabuMove> TabuList);
	/**
	* @brief it is used to make the move cellID1,cellID2 tabu
 	* @param cellID1
 	* @param cellID2
 	* @param TabuList
 	* @param iteration
 	* @param aspirationCriteria
	*/
    void addTabuMove(int cellID1, int cellID2, list<TabuMove> & TabuList, int iteration, bool aspirationCriteria); 
	/**
	* @brief it is used to change the tenure value of a move that already exists in the tabu list
	* @param cellID1
 	* @param cellID2
 	* @param TabuList
 	* @param iteration
	*/
    void changeTenureValue(int cellID1, int cellID2, list<TabuMove> & TabuList, int iteration);
	/**
	* @brief it is used to exchange two cells in the current path if this move is not tabu
	* @param map
	* @param path
	* @param IndexofCellToBeReplaced
	* @param NewCellID
	* @param aspirationCriteria
	* @return the new path after exchanging the two cells if the move is not tabu
	*/
    Path* swapMove(OccupancyGridMap* map, Path* path, int IndexofCellToBeReplaced, int NewCellID, bool & aspirationCriteria);
	/**
	* @brief it is used to insert a new cell in the current path if this move is not tabu
	* @param map
	* @param path
	* @param IndexofCellToBeReplaced
	* @param NewCellID
	* @param aspirationCriteria
	* @return the new path after insertion of the new cell if the move is not tabu
	*/
    Path* insertMove(OccupancyGridMap* map, Path* path, int IndexofCellToBeReplaced, int NewCellID, bool & aspirationCriteria);
	/**
	* @brief it is used to remove a cell from the current path if this move is not tabu
	* @param map
	* @param path
	* @param IndexofCellToBeRemoved
	* @param aspirationCriteria
	* @return the new path after removing the cell if the move is not tabu
	*/
    Path* removeMove(OccupancyGridMap* map,Path* path,int IndexofCellToBeRemoved, bool & aspirationCriteria);
	/**
	* @brief it generates the unvisited common neighbors of two cells
	* @param map
	* @param path
	* @param cellID1
	* @param cellID2
	* @return  a vector that contains the neighbor cells
	*/
    vector <int> findUnvisitedCommonNeighbors(OccupancyGridMap* map, Path* path,  int cellID1, int cellID2);
	/**
	* @brief it generates the unvisited common neighbors of two cells
	* @param map
	* @param currentBloc
	* @param previousBestBlocs
	* @param cellID1
	* @param cellID2
	* @return a vector that contains the neighbor cells
	*/
    vector <int> findUnvisitedCommonNeighborsInBloc(OccupancyGridMap* map, Path* currentBloc, vector<Path*> previousBestBlocs,  int cellID1, int cellID2);
	/**
	* @brief it is to generate the best neighbor from a set of neighbors
	* @param map
	* @param neighborhood
	* @return index of the best neighbor in the vector
	*/
    int findBestNeighbor(OccupancyGridMap* map, vector<Move> neighborhood);
	/**
	* @brief it is to generate the best path from a set of candidate paths
	* @param map
	* @param paths
	* @return index of the best path in the vector
	*/
    int findBestPath(OccupancyGridMap* map, vector<Path*> paths);
	/**
	* @brief it is used to generate N paths after N iterations
	* @param map
	* @param initialPath
	* @param bestExecutionTime
	* @return best path found by tabu search
	*/
    Path* findPath(OccupancyGridMap* map, Path* initialPath, timespec & bestExecutionTime);
	/**
	* @brief it is used to update the tabu list in and out after an interation
	* @param TLIN
	* @param TLOUT
	* @param iteration
	*/
    void updateTabuLists(list<TabuMove> & TLIN, list<TabuMove> & TLOUT, int iteration);
	/**
	* @brief it is used to make the tabu move 
	* @param newPaths
	* @param indexBestPathAfterOneIteration
	* @param TLIN
	* @param TLOUT
	* @param iteration
	*/
    void makeTheMoveTabu (vector <Move> newPaths, int indexBestPathAfterOneIteration, list<TabuMove> & TLIN, list<TabuMove> & TLOUT, int iteration);
	/**
	* @brief it is used to generate path between the start and goal cells accrosing randomly chosen intermediate cell
	* @param map
	* @param Radius
	* @param startCell
	* @param goalCell
	* @return path
	*/
    Path* diversification(OccupancyGridMap* map,int Radius,int startCell, int goalCell);

    
   
  
private:
    Path* bestPath; //!<
    list<TabuMove> tabuListIn; //!<
    list<TabuMove> tabuListOut; //!<
    int tabuListSize; //!<
    int tenure; //!<
    
   
};

#endif
