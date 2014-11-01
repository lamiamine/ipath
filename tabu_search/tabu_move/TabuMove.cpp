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


#include "TabuMove.h"

  //default constructor to initialize path
  TabuMove::TabuMove(int cellS, int cellG,  int tenure, int iteration){
    setFromCell(cellS);
    setToCell(cellG);
    setTenure1(tenure);
    setDateOfExpiration (iteration);
  }
   TabuMove::TabuMove(void){
   }
  //destructor
  TabuMove::~TabuMove(void){
  }
  
  //Define Mutators
  void TabuMove::setFromCell(int cellS){
    fromCell = cellS;
  }
  void TabuMove::setToCell(int cellG){
    toCell = cellG;
  }
  void TabuMove::setTenure1(int t){
     tenure = t;
  }
  void TabuMove::setDateOfExpiration(int iteration){
     dateOfExpiration = iteration;
  }
  
  //Define Accessors
  int TabuMove::getFromCell(){
    return fromCell;
  }
  int TabuMove::getToCell(){
    return toCell;
  }
  int TabuMove::getTenure1(){
    return tenure;
  }
  int TabuMove::getDateOfExpiration(){
     return dateOfExpiration ;
  }
