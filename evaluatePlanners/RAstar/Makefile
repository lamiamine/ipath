CC=g++
CFLAGS=-W -Wall
LDFLAGS=-lrt

DIR_OBJ= ../../../path_planners/lib/
BINDIR = bin

SRC =  evaluateRASTAR.cpp\
       generic_path_planner/GenericPathPlanner.cpp\
       maps/Map.cpp\
       maps/OccupancyGridMap.cpp\
       path/Path.cpp\
       tabu_search/TabuSearch.cpp\
       tabu_search/tabu_move/TabuMove.cpp\
       genetic_algorithm/GA.cpp\
       genetic_algorithm/CrossPoint.cpp\
       genetic_algorithm/Population.cpp\
       AStar/AStar.cpp\
       RAstar/RAStar.cpp\
       ACO/ACO.cpp\
       ACO/Ant.cpp\
       ACO/GenericACO.cpp\
       ACO/pheromone.cpp\
       evaluatePlanners/evaluatePlanners.cpp

     
EXEC=main
OBJS = $(SRC:.cpp=.o)
DIR_OBJS=$(addprefix $(DIR_OBJ), $(OBJS))
 
all: $(EXEC)

main: $(DIR_OBJS)
	$(CC) -o $(BINDIR)/$@ $^ $(LDFLAGS)
 
$(DIR_OBJ)%.o: $(DIR)%.cpp
	$(CC) -o $@ -c $< $(CFLAGS)
