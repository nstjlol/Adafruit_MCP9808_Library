// #include <Wire.h>
// #include <Adafruit_MCP9808.h>

Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

void printTemp(float c, float f);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Serial Started.");

  while (!tempsensor.begin(MCP9808_ADDR)) {
    Serial.println("Can't find MCP9808. Retrying c:\Users\nick_\OneDrive\Documents\Arduino\libraries\Adafruit_MCP9808_Library\Adafruit_MCP9808.cppin 2 seconds.");
    delay(2000);
  }

  Serial.println("MCP9808 found!");

  tempsensor.setResolution(2);

  Serial.println("Setup complete.");
}

void loop() {
  Serial.println("Waking up MCP9808");
  tempsensor.wake();

  float c = tempsensor.readTempC();
  float f = tempsensor.readTempF();
  printTemp(c, f);

  delay(1800);

  Serial.println("Shuting down MCP9808...");
  tempsensor.shutdown();
  Serial.println();
  delay(200);
}

void printTemp(float c, float f) {
  Serial.print("Temp:");
  Serial.print(c); Serial.print("*C\t and ");
  Serial.print(f); Serial.println("*F");
}
