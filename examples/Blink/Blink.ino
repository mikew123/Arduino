
void ledInitTraffic()
{
  pinMode(6, OUTPUT); // Green
  pinMode(7,  OUTPUT);  // Yellow
  pinMode(8,  OUTPUT);  // Red
}


void ledTraffic(bool G, bool Y, bool R)
{
  digitalWrite(6, G); // Green
  digitalWrite(7,  Y); // Yellow
  digitalWrite(8,  R); // Red
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
