
/* 
 * Mini Project 2
 * 2. Implement the Wavefront algorithm using a potential function of own choosing
 * BruteForce approach
 */

#include <iostream> 
#include <fstream>
#include "Image.hpp"
#include "PPMLoader.hpp"

using namespace rw::sensor;
using namespace rw::loaders;

int main(int argc, char * * argv) {

	int **array_str;

	std::string filename(argv[1]);

	std::cout << "Loading img..." << std::endl;
	Image * img = PPMLoader::load(filename);

	const int xHeight = img->getHeight();
	const int xWidth = img->getWidth();
	const int robotSize = xHeight*xWidth;

	std::cout << "Allocate the array..." << std::endl;
	
	// Allocation of the array
	array_str = new int*[xHeight];	// New instans

	// Initialization of each position
	for (int i = 0; i < xHeight; ++i)
	{
		array_str[i] = new int[xWidth];
		for (int j = 0; j < xWidth; ++j)
		{
			if (img->getPixelValuei(j, i, 0) != 0)
				array_str[i][j] = 0;
			else
				array_str[i][j] = 1;
		}
	}
	
	// Set the position of the goal and the robot
	array_str[85][9] = robotSize; 	// Robot
	array_str[874][832] = 2;	// Goal
	
	std::cout << "\nGrap a Coffee, this is going to take a 'While' (or two)..." << std::endl;

	// Generating the WaveFront
	int xWave, yWave;
	bool foundWave = true;
	int currentWave = 2; // First wave is the value of the goal
	
	/* 
	 * THE PRINCIPLE IN THIS IMPLEMENTATION OF THE WAVEFRONT
	 * 
	 * A while-loop goes through each pixel of the image and checks
	 * if the neighboring pixels in the directions UP, DOWN, LEFT and RIGHT.
	 * 
	 * If the neighboring pixel is not obstacle (value less than 0), then the 
	 * pixel-value is set to the number number of wave iterations, plus one. 
	 * Thus, in the next wave it is the neighboring pixel, which neighboring pixels 
	 * is being checked, and so on, until there is no pixels left to check.
	 * 
	 */
	
	while (foundWave == true) // Look for next wave
	{
		foundWave = false;
		for (int y = 0; y < xWidth; y++)
		{
			for (int x = 0; x < xHeight; x++)
			{
				if (array_str[x][y] == currentWave)
				{
					foundWave = true;
					xWave = x;
					yWave = y;
					
					// Left direction
					if (xWave > 0) // Check if the robot hits an obstacle
					  if (array_str[xWave - 1][yWave] == 0) // Check if there is an obstacle
					    array_str[xWave - 1][yWave] = currentWave + 1; // Assign new value
					
					// A similar algorithm is repeated for each direction, so they er not commented.

					  // Right direction
					if (xWave < (xHeight - 1))
					  if (array_str[xWave + 1][yWave] == 0)
					    array_str[xWave + 1][yWave] = currentWave + 1;
					
					  // Down direction
					if (yWave > 0)
					  if (array_str[xWave][yWave - 1] == 0)
					    array_str[xWave][yWave - 1] = currentWave + 1;
					  
					  // Up direction
					if (yWave < (xWidth - 1))
					  if (array_str[xWave][yWave + 1] == 0)
					    array_str[xWave][yWave + 1] = currentWave + 1;
				}
			}
		}
		currentWave++; // Value for the next wave
	}
	
	// Print the Wavemap values in a .txt, to see the 
	std::ofstream myfile;
	myfile.open ("wavemap.txt");
		for(int i = 0; i < xWidth; ++i)
			{
				for(int j = 0; j < xHeight; ++j)
				{
					myfile << array_str[j][i] << "\t";
				}
				myfile << "\n";
			}
			myfile << "\n";
	myfile.close();

	int xRobot, yRobot;

	// Search the map for the locations of the robot
	for (int x = 0; x < xHeight; x++)
	{
		for (int y = 0; y < xWidth; y++)
		{
			if (array_str[x][y] == robotSize)
			{
				xRobot = x;
				yRobot = y;
			}
		}
	}

	int xPos = xRobot, yPos = yRobot; 
	int currentDir = 0;
	int nextDir = 0;
	int current_low = robotSize;
	int moves = 0;
	
	/*
	 * Find the path by looking at the value of the neighboring pixel and finding the
	 * next direction from which value that is the highest. 
	 */
	while (current_low > 2)
	{
		current_low = robotSize;
		nextDir = currentDir;
		int xNext = 0;
		int yNext = 0;

		// Left direction
		if (xPos > 0) // 
		  if (array_str[xPos - 1][yPos] < current_low && array_str[xPos - 1][yPos] != 1)
		  {
			  current_low = array_str[xPos - 1][yPos]; // Set next number
			  nextDir = 3; //Set next Direction
			  xNext = xPos - 1;
			  yNext = yPos;
			  // A similar algorithm is repeated for each direction, so they er not commented.
		  }

		// Right direction
		if (xPos < (xHeight - 1))
		  if (array_str[xPos + 1][yPos] < current_low && array_str[xPos + 1][yPos] != 1) 
		  {
			  current_low = array_str[xPos + 1][yPos]; 
			  nextDir = 1; 
			  xNext = xPos + 1;
			  yNext = yPos;
		  }

		// Down direction
		if (yPos > 0)
		  if (array_str[xPos][yPos - 1] < current_low && array_str[xPos][yPos - 1] != 1)
		  {
			  current_low = array_str[xPos][yPos - 1]; 
			  nextDir = 2;
			  xNext = xPos;
			  yNext = yPos - 1;
		  }

		// Up direction
		if (yPos < (xWidth - 1))
		  if (array_str[xPos][yPos + 1] < current_low && array_str[xPos][yPos + 1] != 1) 
		  {
			  current_low = array_str[xPos][yPos + 1];
			  nextDir = 0; 
			  xNext = xPos;
			  yNext = yPos + 1;
		  }
		
		xPos = xNext;
		yPos = yNext;
		array_str[xPos][yPos] = 9999; // Mark where the robot have been
		img->setPixel8U(yPos, xPos, 125); // Color the robots path to the goal

		// Making the path from the nextDir values
		while (currentDir != nextDir)
		{
			if (currentDir > nextDir)
			{
				currentDir--;
			}
			else if (currentDir < nextDir)
			{
				currentDir++;
			}
		}

		moves++; // Count the moves of the path
	}

	std::cout << "\nNumbers of moves: " << moves << std::endl;

	std::cout << "Saving image..." << std::endl;
	
	// Save image
	img->saveAsPGM("testout.pgm");

	// Clean up; deallocate the array and pointer
	for (int i = 0; i < xHeight; i++){
		delete[] array_str[i];		// Delete each row
	}
	delete[] array_str;		// Delete array pointer

	delete img;
}
