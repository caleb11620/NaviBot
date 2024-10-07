#include <Arduino.h>
// Include necessary library headers
// Note: You may need to implement or find Arduino-compatible versions of these
#include <microTuple.h>
#include "Bitmap.h"
#include "Astar.h"

struct step {
  int angle;
  int distance {1};
  char direction;
};

// Function prototypes
MicroTuple<char, int, String> calculateDirectionAngleAndHeading(int x1, int y1, int x2, int y2, const String& currentHeading);
void sdInit();

// SD Card
File myFile;
File maze_bmp;
File cropped_bmp;
const int sdPin = 10; // update to match connection

void setup() {
  Serial.begin(9600);
  Serial.println("Hello, from Arduino maze solver!");

  // SD Card init
  sdInit();

  // Initialize your bitmap and A* objects here
  Bitmap bmp;
  Astar astar;

  // read bitmap
  if (bmp.read(maze_bmp)) {
    Serial.println("Bitmap successfully read.");
  } else {
    Serial.println("Error reading bitmap.");
  }
  maze_bmp.close();

  bmp.removeEmptyRowsAndColumns();
  cropped_bmp = SD.open("croppedmaze.bin", FILE_WRITE);
  if (bmp.write(cropped_bmp)) {
    Serial.println("cropped_bmp created");
  }
  cropped_bmp.close();

  Serial.print("Adjusted bitmap width: ");
  Serial.print(bmp.getWidth());
  Serial.print(" height: ");
  Serial.println(bmp.getHeight());

  auto astardata = bmp.getData();
  astar.interpretBitmap(astardata);

  Node* start = astar.determineStartNode();
  Node* exit = astar.determineGoalNode();

  Serial.print("startNode x:");
  Serial.print(start->x);
  Serial.print(" y:");
  Serial.println(start->y);
  Serial.print("exitNode x:");
  Serial.print(exit->x);
  Serial.print(" y:");
  Serial.println(exit->y);

  std::vector<Node*> path = astar.algorithm(start, exit);
  std::vector<step> solution;

  astar.printGrid(path);

  char direction;
  int angle;
  String newHeading = "N";
  String heading = newHeading;
  
  Serial.println("Path vector: (x,y)");
  for (int i = 0 ; i < path.size()-1; ++i) {
    Serial.print("(");
    Serial.print(path[i]->x);
    Serial.print(",");
    Serial.print(path[i]->y);
    Serial.print(") -> (");
    Serial.print(path[i+1]->x);
    Serial.print(",");
    Serial.print(path[i+1]->y);
    Serial.print(") ");
    
    MicroTuple<char, int, String> results = calculateDirectionAngleAndHeading(path[i]->x, path[i]->y, path[i+1]->x, path[i+1]->y, heading);
    direction = results.get<0>();
    angle = results.get<1>();
    newHeading = results.get<2>();
    heading = newHeading;
    
    Serial.print("Direction: ");
    Serial.print(direction);
    Serial.print(", Angle: ");
    Serial.print(angle);
    Serial.print(", New Heading: ");
    Serial.println(newHeading);
    
    step val;
    val.direction = direction;
    val.angle = angle;
    val.distance = 1;
    solution.push_back(val);
  }

  // Simplify consecutive forward steps
  for (int i = 0; i < solution.size()-1; ++i) {
    if (solution[i].direction == 'F' && solution[i].direction == solution[i+1].direction) {
      solution[i].distance++;
      solution.erase(solution.begin() + i+1);
      --i;
    }
  }

  // Print the solution
  for (int i = 0; i < solution.size(); ++i) {
    Serial.print("step ");
    Serial.print(i);
    Serial.print(": angle: ");
    Serial.print(solution[i].angle);
    Serial.print(" direction: ");
    Serial.print(solution[i].direction);
    Serial.print(" distance: ");
    Serial.println(solution[i].distance);
  }

  astar.cleanupGrid();
}

void loop() {
  // The main loop can be left empty or used for repeated tasks
}

// Implement the calculateDirectionAngleAndHeading function here
// You may need to adapt this function to use Arduino's String instead of std::string
// and replace std::map with a simpler data structure if memory is a concern
MicroTuple<char, int, String> calculateDirectionAngleAndHeading(int x1, int y1, int x2, int y2, const String& currentHeading) {
  // Implementation goes here
  // This function will need significant adaptation for Arduino
  // You may need to use simpler data structures instead of std::map
  // and implement your own string comparison logic
}

// Initialization of SD card and log file
void sdInit() {
  Serial.print("Initializing SD Card...");
  if(!SD.begin(sdPin)) {
    Serial.println("Initialization Failure...Check wiring/connection");
    while(1==1);
  }
  Serial.print("Initialization Complete");  
  
  // init files
  myFile = SD.open("log.txt");
  maze_bmp = SD.open("400x400maze.bin");
}