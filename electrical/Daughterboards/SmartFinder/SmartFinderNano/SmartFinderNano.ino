int audioPin = 0; // Analog pin to read from
int audioTemp; // Temp variable for comparison
int maxVol; // Highest analogRead value
int minVol; // Lowest analogRead value
byte finalVol; // Calculated volume to be communicated

void setup() {
  Serial.begin(9600);
}

void loop() {
  maxVol = 0;
  minVol = 1024;
  while (Serial.read() != 'a') {}
  // Loop ten times and populate audioArray with analogRead data
  for (int i = 0; i < 10; i++) {
    // analogRead takes ~100 us
    // Which means this loop runs a little slower than 1 kHz
    // This will encompass approximately one audio cycle
    audioTemp = analogRead(audioPin);
    if (audioTemp > maxVol)
      maxVol = audioTemp;
    if (audioTemp < minVol)
      minVol = audioTemp;
  }
  // Calculate volume in byte formate for serial transmission
  finalVol = (byte)((maxVol - minVol)/4);
  // Send the volume info downstream for higher level processing
  Serial.write(finalVol);
}
