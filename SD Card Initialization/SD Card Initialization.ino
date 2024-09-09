  //Credit to David A. Mellis and Tom Igoe for the basis of this program on https://www.arduino.cc/en/Tutorial/LibraryExamples/ReadWrite
  #include <SPI.h>
  #include <SD.h>

  File myFile;
  const int chipSelect = 10; //change this to match your SD shield or module;

void setup() {
  Serial.begin(9600);
  
  Serial.print("Initializing SD Card...");
  if(!SD.begin(10)){ //change 4 to whatever the CS pin is
    Serial.println("Initialization Failure...Check wiring/connection");
    while(1==1);
  }
  Serial.print("Initialization Complete :)");

  myFile = SD.open("test.txt", FILE_WRITE); //creates test.txt file and opens it for writing

  if(myFile){
    Serial.print("Writing to test.txt...");
    myFile.println("testing 1, 2, 3.");
    myFile.close(); // close the file and saves to the SD
    Serial.println("done.");
    return;
  }
  else{
    Serial.print("Failed to open test.txt");
  }

  myFile = SD.open("test.txt"); //opens existing test.txt for reading
  if(myFile){
    Serial.println("Test.txt");
    while(myFile.available()){ //while loop needed as read() only reads one character at a time
      Serial.write(myFile.read());
    }
    myFile.close();
  }
  else{
    Serial.println("Error opening test.txt");
  }

}

void loop() {
  // put your main code here, to run repeatedly:

}
