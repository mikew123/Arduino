
void ledInitTraffic()
{
  pinMode(D10, OUTPUT); // Green
  pinMode(D9,  OUTPUT);  // Yellow
  pinMode(D8,  OUTPUT);  // Red
}


void ledTraffic(bool G, bool Y, bool R)
{
  digitalWrite(D10, G); // Green
  digitalWrite(D9,  Y); // Yellow
  digitalWrite(D8,  R); // Red
}

void setup() {
  ledInitTraffic();
}


void loop() {
  ledTraffic(1,0,0);
  delay(250);
  ledTraffic(1,1,0);
  delay(250);
  ledTraffic(1,1,1);
  delay(250);
  ledTraffic(0,1,1);
  delay(250);
  ledTraffic(0,0,1);
  delay(250);
}
