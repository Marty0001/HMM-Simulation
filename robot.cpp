#include <iostream>
#include <iomanip>
#include<cstdlib>
#include <ctime>
#include<vector>
#include "HMM.h"
/*
cd /mnt/c/College\ Y4/CIS\ 479/P2/Robot
g++ -std=c++14 robot.cpp -o robot
./robot
*/

Robot robot;//robot can only sense and predict where could be after it moves, given observations from its sensor

void pickStartLocation(int &row, int &column) {

	while (std::cout << "Row Column: ") {

		std::cin >> row >> column;

		if (!(std::cin.good()) || row < 0 || column < 0 || row >= robot.getMaxRow() || column >= robot.getMaxColumn() || robot.badStart(row,column)) {

			std::cout << "invalid location!" << std::endl;
			std::cin.clear();
			std::cin.ignore(100, '\n');
		}
		else {
			std::cout << std::endl;
			break;
		}
	}
}

//has achance to return 3 values for straight, left, or right
int simulateDirft() {

	int random = 1 + (rand() % 100);

	//10% chance to drift right
	if (random <= 10)
		return 1;

	//15% chance to drift left
	else if (random > 10 && random <= 25)
		return 2;
	//75% chance to go straight
	else
		return 3;
}


bool simulateSensorError(bool obstacle) {

	int random = 1 + (rand() % 100);

	//if theres an obstacle, theres a 10% chance to not see it
	if (obstacle && random <= 10) 
		obstacle = false;

	//if theres no obstacle, theres a 5% chance to see an obstacle
	else if (!obstacle && random <= 5) 
		obstacle = true;

	return obstacle;
}

void move(int &row, int &column, int direction, std::vector<int> &movementLog) {

	int drift = simulateDirft();//gets result on if the robot will drift or not
	Robot humanEyes;//use humanEyes to look at obstacles just to make more seperation so the robot never gets its location as input
	int prevRow = row, prevColumn = column;

	robot.predict(direction);//robot predicts where it could be after it moves in the given direction, it also considers the potential for drifting
	robot.displayProbabilites();

	switch (direction) {
	case 1:
		std::cout << "THE ROBOT MOVED WEST" << std::endl;
		break;

	case 2:
		std::cout << "THE ROBOT MOVED NORTH" << std::endl;
		break;

	case 3:
		std::cout << "THE ROBOT MOVED EAST" << std::endl;
		break;

	case 4:
		std::cout << "THE ROBOT MOVED SOUTH" << std::endl;
		break;
	}

	//right
	if (drift == 1) {
		if (direction != 4)
			direction++;
		else
			direction--;
		std::cout << "THE ROBOT DRIFTED TO THE RIGHT" << std::endl;
	}
	//left
	else if (drift == 2) {
		if (direction != 1)
			direction--;
		else
			direction++;
		std::cout << "THE ROBOT DRIFTED TO THE LEFT" << std::endl;
	}
	
	//if theres an obstacle in the direction the robot wants to go, it will stay in the same spot. Robot doesnt know if it hits an obstacle

	//move west
	if (direction == 1) {
		if (!humanEyes.lookWest(row, column)) {
			column--;
		}
	}
	//move north
	if (direction == 2) {
		if (!humanEyes.lookNorth(row, column)) {
			row--;
		}
	}
	//move east
	if (direction == 3) {
		if (!humanEyes.lookEast(row, column)) {
			column++;
		}
	}
	//move south
	if (direction == 4) {
		if (!humanEyes.lookSouth(row, column)) {
			row++;
		}		
	}

	if (prevRow == row && prevColumn == column)
		std::cout << "THE ROBOT BOUNCED OFF AN OBSTACLE" << std::endl;
	else
		movementLog.push_back(direction);
}

int sensors(int &row, int &column) {
	//simulate human knowing where the robot is and if there is obstacles around it
	Robot humanEyes;

	//these are true of there is an obsacle in the given direction
	bool west = humanEyes.lookWest(row, column);
	bool north = humanEyes.lookNorth(row, column);
	bool east = humanEyes.lookEast(row, column);
	bool south = humanEyes.lookSouth(row, column);

	//sensors have a chance to sense its surroundings incorrectly
	west = simulateSensorError(west);
	north = simulateSensorError(north);
	east = simulateSensorError(east);
	south = simulateSensorError(south);

	//robot only gets sensor information which could be wrong, doesn't know its location
	robot.sense(west, north, east, south);
	robot.displayProbabilites();

	//robot chooses a random direction to go in if it thinks its open
	//allBlocked is for very rare chance that it senses obstacles in all directions and can't decide where to go
	int direction = 0, allBlocked = 0;
	while (allBlocked < 50 && (direction = 1 + (rand() % 4))) {

		if (!west && direction == 1)
			break;
		else if (!north && direction == 2)
			break;
		else if (!east && direction == 3)
			break;
		else if (!south && direction == 4)
			break;
		else
			allBlocked++;
	}
	return direction;
}

int main() {

	srand(time(NULL));
	int row = 0, column = 0;//user chooses where to place robot
	int direction = 0;//west = 1 north = 2 east = 3 south = 4
	int actions = 1;//snesing and moving counts as an action, start at 1 because it senses before the loop starts
	std::vector<int> movementLog;//keep track of what directions the robot moved to see if its belief is accurate


	robot.mazeLayout();

	std::cout << "Enter starting location (robot will not know)" << std::endl;

	pickStartLocation(row, column);//validate user input


	direction = sensors(row, column);//sensors help robot decide which direction to go and gives evidence for possible locations

	while (!robot.knowsLocation()) {

		move(row, column, direction, movementLog);//robot moves and predicts where it will end up

		direction = sensors(row, column);//sensors help robot decide which direction to go and gives evidence for possible locations

		actions+=2;
	}

	robot.locationBelief();//robot says where it thinks it is

	std::cout << "\nACTIONS: " << actions << std::endl;

	std::cout << "FROM START LOCATION, THE ROBOT HAS MOVED:" << std::endl;

	for (int d : movementLog) {

		switch (d) {
		case 1:
			std::cout << "WEST" << std::endl;
			break;

		case 2:
			std::cout << "NORTH" << std::endl;
			break;

		case 3:
			std::cout << "EAST" << std::endl;
			break;

		case 4:
			std::cout << "SOUTH" << std::endl;
			break;
		}
	}
}