#include <Arduino.h>
// Include necessary library headers
#include <microTuple.h>
#include "Bitmap.h"
#include "Astar.h"
#include <map>

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
  Serial.println("Running setup");
  // SD Card init
  sdInit();

  // Initialiaze bmp and astar
  Bitmap bmp;
  Astar astar;

  // Read bitmap from SD
  if (bmp.read(maze_bmp)) {
    Serial.println(F("Bitmap successfully read."));
  } else {
    Serial.println(F("Error reading bitmap."));
  }
  maze_bmp.close();

  // Crop empty space from bitmap for readability
  bmp.removeEmptyRowsAndColumns();
  cropped_bmp = SD.open("croppedmaze.bin", FILE_WRITE);
  if (bmp.write(cropped_bmp)) {
    Serial.println(F("cropped_bmp created"));
  }
  cropped_bmp.close();

  // Check new width and height
  /*
  Serial.print("Adjusted bitmap width: ");
  Serial.print(bmp.getWidth());
  Serial.print(" height: ");
  Serial.println(bmp.getHeight());
  */

  // Initialize grid data in A* class
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
    
    step val = {angle, 1, direction};
    //val.direction = direction;
    //val.angle = angle;
    //val.distance = 1;
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

// transform coordinate directions into step by step angle, direction, distance
MicroTuple<char, int, String> calculateDirectionAngleAndHeading(int x1, int y1, int x2, int y2, const String& currentHeading) {
  int dx = x2 - x1;
  int dy = (y1 - y2);

  const std::map<std::pair<int, int>, String> movements = {
    {{0, 1}, "N"}, {{1, 1}, "NE"}, {{1, 0}, "E"}, {{1, -1}, "SE"},
    {{0, -1}, "S"}, {{-1, -1}, "SW"}, {{-1, 0}, "W"}, {{-1, 1}, "NW"}
  };

  // Find the new heading
  auto it = movements.find({dx, dy});
  if (it == movements.end()) {
    return {'X', -1, currentHeading};  // Invalid movement
  }
  String newHeading = it->second;
  
  // If the heading hasn't changed, it's a forward movement
  if (newHeading == currentHeading) {
    return {'F', 0, newHeading};
  }
  
  // Define the angle differences between directions
  const std::map<std::pair<String, String>, std::pair<char, int>> directionChanges = {
    {{"N", "NE"}, {'R', 45}},  {{"N", "E"}, {'R', 90}},   {{"N", "SE"}, {'R', 135}},
    {{"N", "S"}, {'R', 180}},  {{"N", "SW"}, {'L', 135}}, {{"N", "W"}, {'L', 90}},
    {{"N", "NW"}, {'L', 45}},  {{"NE", "E"}, {'R', 45}},  {{"NE", "SE"}, {'R', 90}},
    {{"NE", "S"}, {'R', 135}}, {{"NE", "SW"}, {'R', 180}},{{"NE", "W"}, {'L', 135}},
    {{"NE", "NW"}, {'L', 90}}, {{"NE", "N"}, {'L', 45}},  {{"E", "SE"}, {'R', 45}},
    {{"E", "S"}, {'R', 90}},   {{"E", "SW"}, {'R', 135}}, {{"E", "W"}, {'R', 180}},
    {{"E", "NW"}, {'L', 135}}, {{"E", "N"}, {'L', 90}},   {{"E", "NE"}, {'L', 45}},
    {{"SE", "S"}, {'R', 45}},  {{"SE", "SW"}, {'R', 90}}, {{"SE", "W"}, {'R', 135}},
    {{"SE", "NW"}, {'R', 180}},{{"SE", "N"}, {'L', 135}}, {{"SE", "NE"}, {'L', 90}},
    {{"SE", "E"}, {'L', 45}},  {{"S", "SW"}, {'R', 45}},  {{"S", "W"}, {'R', 90}},
    {{"S", "NW"}, {'R', 135}}, {{"S", "N"}, {'R', 180}},  {{"S", "NE"}, {'L', 135}},
    {{"S", "E"}, {'L', 90}},   {{"S", "SE"}, {'L', 45}},  {{"SW", "W"}, {'R', 45}},
    {{"SW", "NW"}, {'R', 90}}, {{"SW", "N"}, {'R', 135}}, {{"SW", "NE"}, {'R', 180}},
    {{"SW", "E"}, {'L', 135}}, {{"SW", "SE"}, {'L', 90}}, {{"SW", "S"}, {'L', 45}},
    {{"W", "NW"}, {'R', 45}},  {{"W", "N"}, {'R', 90}},   {{"W", "NE"}, {'R', 135}},
    {{"W", "E"}, {'R', 180}},  {{"W", "SE"}, {'L', 135}}, {{"W", "S"}, {'L', 90}},
    {{"W", "SW"}, {'L', 45}},  {{"NW", "N"}, {'R', 45}},  {{"NW", "NE"}, {'R', 90}},
    {{"NW", "E"}, {'R', 135}}, {{"NW", "SE"}, {'R', 180}},{{"NW", "S"}, {'L', 135}},
    {{"NW", "SW"}, {'L', 90}}, {{"NW", "W"}, {'L', 45}}
  };
  
  // Find the direction change
  auto change = directionChanges.find({currentHeading, newHeading});
  if (change != directionChanges.end()) {
    return {change->second.first, change->second.second, newHeading};
  }
  
  // If no change found (shouldn't happen with a complete directionChanges map)
  return {'X', -1, newHeading};
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