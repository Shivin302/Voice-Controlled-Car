
int bit7 = 3;           // MSB, P3.4
int bit6 = 4;           // P3.3
int bit5 = 5;           // P1.6
int bit4 = 6;           // P6.6
int bit3 = 7;           // P3.2
int bit2 = 8;           // P2.7
int bit1 = 9;           // P4.2
int bit0 = 10;          // LSB, P4.1

int val = 0;
int added = 1;

void setup()
{
  // Set pins to output mode
  pinMode(bit7, OUTPUT);
  pinMode(bit6, OUTPUT);
  pinMode(bit5, OUTPUT);
  pinMode(bit4, OUTPUT);
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

  // If bit 4 is set in val, turn bit4 high
  if(val & 16)
    digitalWrite(bit4, HIGH);
  else
    digitalWrite(bit4, LOW);

  // If bit 5 is set in val, turn bit5 high
  if(val & 32)
    digitalWrite(bit5, HIGH);
  else
    digitalWrite(bit5, LOW);

  // If bit 6 is set in val, turn bit6 high
  if(val & 64)
    digitalWrite(bit6, HIGH);
  else
    digitalWrite(bit6, LOW);

  // If bit 7 is set in val, turn bit7 high
  if(val & 128)
    digitalWrite(bit7, HIGH);
  else
    digitalWrite(bit7, LOW);

  // Range of values is 0 to 255 for 8 bit DAC
  if(val == 255)
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
