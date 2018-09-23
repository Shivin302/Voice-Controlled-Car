int bit3 = 7;
int bit2 = 8;           // P2.7
int bit1 = 9;           // P4.2
int bit0 = 10;          // LSB, P4.1

int val = 0;
int added = 1;

void setup()
{
  // Set pins to output mode
  pinMode(bit3, OUTPUT);
  pinMode(bit2, OUTPUT);
  pinMode(bit1, OUTPUT);
  pinMode(bit0, OUTPUT);

  reset_blinker();
}

void loop()
{
  // If LSB is set in val, turn bit0 high
  if(val & 1)
    digitalWrite(bit0, HIGH);
  else
    digitalWrite(bit0, LOW);

  // If bit 1 is set in val, turn bit1 high
  if(val & 2)
    digitalWrite(bit1, HIGH);
  else
    digitalWrite(bit1, LOW);

  // If bit 2 is set in val, turn bit2 high
  if(val & 4)
    digitalWrite(bit2, HIGH);
  else
    digitalWrite(bit2, LOW);

  // If bit 3 is set in val, turn bit3 high
  if(val & 8)
    digitalWrite(bit3, HIGH);
  else
    digitalWrite(bit3, LOW);

  // Range of values is 0 to 15 for 4 bit DAC
  if(val == 15)
    added = -1;
  if (val == 0)
    added = 1;

  val = val + added;

  delay(10);
}

// Reset success indicator
void reset_blinker(){
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(RED_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, LOW);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(GREEN_LED, LOW);
}