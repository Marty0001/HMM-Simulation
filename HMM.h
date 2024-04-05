#pragma once

#include <iostream>
#include <iomanip>

class Robot {

private:

	constexpr static int MAX_ROW = 7;
	constexpr static int MAX_COLUMN = 7;

	const bool maze[MAX_ROW][MAX_COLUMN] = {
		//   0 1 2 3 4 5 6 
			{0,0,0,0,0,0,0},// 0
			{0,1,0,1,0,1,0},// 1
			{0,0,0,0,0,1,0},// 2
			{0,0,0,1,0,0,0},// 3
			{0,1,0,0,0,0,0},// 4
			{0,1,0,1,0,1,0},// 5
			{0,0,0,0,0,0,0},// 6 
	};;

	double HMM[MAX_ROW][MAX_COLUMN];

	//Transitional Probabilty
	double getDirectionProbability(int currentDir, int targetDir, bool obstacle) {

		/*
		returns probabilty to get to square 'x' from currentDirection given the targetDirection

		Example:

		if the square is top left corner (0,0), how can we get to that sqaure if the target direction is East(3)?
		look in every direction of the square, this is the currentDir:

		if the currentDir is west(1), then its impossible to get to 0,0 from the west because its an obstacle, so P = 0
		if the currentDir is north(2), the north square is an obstacle, and the robot can bounce off the obstacle if it goes left, so P = 15
		if the currentDir is east(3), its impossible because you cant move backwards from the targetDir, so P = 0
		if the currentDir is south(4), since its an open sqaure, it can go from (1,0) to (0,0) if it drifts left so P = 15
		*/

		//if current direction is open
		if (!obstacle) {
			//straight: cant go backwards from an open square so make sure current and target dont equal. If the mods of both are equal though, its opposite direcitons
			if (currentDir != targetDir && currentDir % 2 == targetDir % 2) {
				return 75;
			}
			//left:from an open sqaure, if the targetDir is 1 less than the currentDir, its left, unless its 1 and 4
			else if ((currentDir == 1 && targetDir == 4) || (currentDir == targetDir + 1)) {
				return 15;
			}
			//right:
			else if ((currentDir == 4 && targetDir == 1) || (currentDir == targetDir - 1)) {
				return 10;
			}
			else
				return 0;
		}
		//if current direction is an obstacle
		else {
			//straight: if theres an obstacle in the target direction, it will bounce off
			if (currentDir == targetDir) {
				return 75;
			}
			//left:opposite rules as open sqaure since robot bounces off obstacle
			else if ((currentDir == 4 && targetDir == 1) || (currentDir == targetDir - 1)) {
				return 15;
			}
			//right:
			else if ((currentDir == 1 && targetDir == 4) || (currentDir == targetDir + 1)) {
				return 10;
			}
			else
				return 0;
		}
	}

	//Filtering
	double getSenseProbability(bool observation, bool obstacle) {

		if (observation && obstacle) {      //%90 to observe an obstacle correctly
			return 90;
		}
		else if (observation && !obstacle) {//5% to see an obstacle on an open sqaure
			return 5;
		}
		else if (!observation && obstacle) {//10% to see an open sqaure on an obstacle
			return 10;
		}
		else {                               //95% to observe an open square correctly 
			return 95;
		}
	}

public:

	Robot() {

		for (int i = 0; i < MAX_ROW; i++) {
			for (int j = 0; j < MAX_COLUMN; j++) {

				if (maze[i][j])
					HMM[i][j] = 0;

				else
					HMM[i][j] = 2.5;
			}
		}
	}

	//lookDirection returns true if the square to the 'Direction' of the current sqaure is an obstacle
	bool lookWest(int row, int column) {
		if (column == 0 || maze[row][column - 1]) {
			return true;
		}
		return false;
	}

	bool lookNorth(int row, int column) {
		if (row == 0 || maze[row - 1][column]) {
			return true;
		}
		return false;
	}

	bool lookEast(int row, int column) {
		if (column == 6 || maze[row][column + 1]) {
			return true;
		}
		return false;
	}

	bool lookSouth(int row, int column) {
		if (row == 6 || maze[row + 1][column]) {
			return true;
		}
		return false;
	}

	void displayProbabilites() {

		for (int i = 0; i < MAX_ROW; i++) {
			for (int j = 0; j < MAX_COLUMN; j++) {

				if (HMM[i][j] > 0)
					std::cout << std::setprecision(2) << std::fixed << std::setw(6) << HMM[i][j];
				else
					std::cout << std::setw(6) << "####";
			}
			std::cout << std::endl << std::endl;
		}
		std::cout << std::endl;
	}

	//Sensing: Evidence Conditional Probability P(Zt|St)
	void sense(bool west, bool north, bool east, bool south) {

		if (west)
			std::cout << "Robot: I see an obstacle to the west" << std::endl;
		if (north)
			std::cout << "Robot: I see an obstacle to the north" << std::endl;
		if (east)
			std::cout << "Robot: I see an obstacle to the east" << std::endl;
		if (south)
			std::cout << "Robot: I see an obstacle to the south" << std::endl;

		bool westObstacle = false, northObstacle = false, eastObstacle = false, southObstacle = false;
		double normalize = 0;

		for (int i = 0; i < MAX_ROW; i++) {
			for (int j = 0; j < MAX_COLUMN; j++) {

				if (!maze[i][j]) {//if not an obstacle square, do calculations

					//look in all directions of current sqaure to see where actual obstacles are to compare with the observation
					westObstacle = lookWest(i, j);
					northObstacle = lookNorth(i, j);
					eastObstacle = lookEast(i, j);
					southObstacle = lookSouth(i, j);

					//P(Zt|St) = P(ZW,t|St) P(ZN,t|St) P(ZE,t|St) P(ZS,t|St)
					HMM[i][j] *= getSenseProbability(west, westObstacle);
					HMM[i][j] *= getSenseProbability(north, northObstacle);
					HMM[i][j] *= getSenseProbability(east, eastObstacle);
					HMM[i][j] *= getSenseProbability(south, southObstacle);

					normalize += HMM[i][j];
				}
			}
		}

		//divide each sqaure probabilty by total of all sqaures to normalize, then mult by 100 for percentage value
		for (int i = 0; i < MAX_ROW; i++) {
			for (int j = 0; j < MAX_COLUMN; j++) {

				(HMM[i][j] /= normalize) *= 100;
			}
		}
	}

	//Prediction
	void predict(int direction) {

		switch (direction) {
		case 1: 
			std::cout << "Robot: If I move West, these are my possible locations:" << std::endl;
			break;

		case 2: 
			std::cout << "Robot: If I move North, these are my possible locations:" << std::endl;
			break;

		case 3: 
			std::cout << "Robot: If I move East, these are my possible locations:" << std::endl;
			break;

		case 4: 
			std::cout << "Robot: If I move South, these are my possible locations:" << std::endl;
			break;
		}

		double tempHMM[MAX_ROW][MAX_COLUMN]{};//temp HMM to hold prediction values since it must use the previous observation values to calculate

		//*Sigma* sP(St + 1 | St = s) P(St | Z1 = z1, ..., Zt = zt)
		for (int i = 0; i < MAX_ROW; i++) {
			for (int j = 0; j < MAX_COLUMN; j++) {

				if (!maze[i][j]) {

					if (lookWest(i, j)) {//west square is blocked
						tempHMM[i][j] += getDirectionProbability(1, direction, true) * HMM[i][j];
					}
					else {//west square is open
						tempHMM[i][j] += getDirectionProbability(1, direction, false) * HMM[i][j - 1];
					}

					if (lookNorth(i, j)) {//north square is blocked
						tempHMM[i][j] += getDirectionProbability(2, direction, true) * HMM[i][j];
					}
					else {//north square is open
						tempHMM[i][j] += getDirectionProbability(2, direction, false) * HMM[i - 1][j];
					}

					if (lookEast(i, j)) {//east square is blocked
						tempHMM[i][j] += getDirectionProbability(3, direction, true) * HMM[i][j];
					}
					else {//east square is open
						tempHMM[i][j] += getDirectionProbability(3, direction, false) * HMM[i][j + 1];
					}

					if (lookSouth(i, j)) {//south square is blocked
						tempHMM[i][j] += getDirectionProbability(4, direction, true) * HMM[i][j];
					}
					else {//south square is open
						tempHMM[i][j] += getDirectionProbability(4, direction, false) * HMM[i + 1][j];
					}
				}
			}
		}

		for (int i = 0; i < MAX_ROW; i++) {
			for (int j = 0; j < MAX_COLUMN; j++) {

				HMM[i][j] = tempHMM[i][j] / 100;
			}
		}
	}

	//if a sqaure has a near 100% probabilty, it thinks it knows the location
	bool knowsLocation() {

		for (int i = 0; i < MAX_ROW; i++) {
			for (int j = 0; j < MAX_COLUMN; j++) {

				if (HMM[i][j] > 99) {
					return true;
				}
			}
		}
		return false;
	}

	//get the location of the sqaure with highest probability
	void locationBelief() {

		double maxProbabilty = 0;
		int row = 0, column = 0;

		for (int i = 0; i < MAX_ROW; i++) {
			for (int j = 0; j < MAX_COLUMN; j++) {

				if (HMM[i][j] > maxProbabilty) {
					maxProbabilty = HMM[i][j];
					row = i;
					column = j;
				}
			}
		}

		std::cout << "Robot: I believe I am currently at location " << row << " " << column << std::endl;

	}

	void mazeLayout() {

		std::cout << "                  Maze layout:" << std::endl;

		std::cout << "ROW" << std::endl;

		for (int i = 0; i < MAX_ROW; i++) {
			std::cout << " " << i << "      ";
			for (int j = 0; j < MAX_COLUMN; j++) {

				if (!maze[i][j])
					std::cout << "[ ]  ";
				else
					std::cout << "[#]  ";
			}
			std::cout << std::endl << std::endl;
		}
		std::cout << "COLUMN   0    1    2    3    4    5    6" << std::endl << std::endl;
	}

	int getMaxRow() {
		return MAX_ROW;
	}

	int getMaxColumn() {
		return MAX_COLUMN;
	}

	//check if the user inputed a location ontop of an obstacle
	bool badStart(int row, int column) {
		if (maze[row][column])
			return true;
		return false;
	}
};