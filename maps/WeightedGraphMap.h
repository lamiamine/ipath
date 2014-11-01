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

#include"Map.h"

class WeightedGraphMap : public Map {
 
public:
  
 //default constructor to initialize map
  WeightedGraphMap(int w=0, int h=0, float res=0.0);
  //four-argument constructor to initialize map
  WeightedGraphMap(int w, int h, float res, vector <int> mapVector);
  //constructor that loads a map from a file
  WeightedGraphMap(ofstream mpgMapFile); 
  //Copy constructor for the OccupancyGridMapGridMap
  WeightedGraphMap (OccupancyGridMapGridMap & map);
  
  //setEdgeCost with input parameter index in the map vector
  void setEdgeCost (vector<int>::iterator it);  
  //setEdgeCost input parameter with index (i,j) in the matrix map
  void setEdgeCost (int row, int col);
  
  //Check whether two points are connected
  bool isConnected (int WayPoint1, int WayPoint2 );  
};
