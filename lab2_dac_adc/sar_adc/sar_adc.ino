int baudRate = 9600;   // serial connection speed
int clk_period = 100;    // simulated clock period in ms

int bit3 = 7;           // MSB, P3.2
int bit2 = 8;           // P2.7
int bit1 = 9;           // P4.2
int bit0 = 10;          // LSB, P4.1

int v_in = 2;           // P6.5

int result = 0;

void setup() { 

  // Start a serial connection to send data to the computer
  Serial.begin(baudRate);

  // Setup the digital pins as outputs 
  pinMode(bit3, OUTPUT); 
  pinMode(bit2, OUTPUT); 
  pinMode(bit1, OUTPUT); 
  pinMode(bit0, OUTPUT); 

  // Set the initial values of these pins to LOW (0V)
  digitalWrite(bit3, LOW);
  digitalWrite(bit2, LOW);
  digitalWrite(bit1, LOW);
  digitalWrite(bit0, LOW);
  
  Serial.print("Setup done\n");
  reset_blinker();
}

void loop () {  
  // cycle 1: clean all  
  digitalWrite(bit3, LOW);
  digitalWrite(bit2, LOW);
  digitalWrite(bit1, LOW);
  digitalWrite(bit0, LOW);
  result = 0;
  
  delayMicroseconds(clk_period);
  
  // cycle 2: test bit3
  digitalWrite(bit3, HIGH);
  delayMicroseconds(clk_period/2);
  if (digitalRead(v_in)){
    result += 8;
  } else { 
    digitalWrite(bit3, LOW);
  }
  delayMicroseconds(clk_period/2);
  
  // cycle 3: test bit2
  digitalWrite(bit2, HIGH);
  delayMicroseconds(clk_period/2);
  if (digitalRead(v_in)){
    result += 4;
  } else { 
    digitalWrite(bit2, LOW);
  }
  delayMicroseconds(clk_period/2);
  
  // cycle 4: test bit1
  digitalWrite(bit1, HIGH);
  delayMicroseconds(clk_period/2);
  if (digitalRead(v_in)){
    result += 2;
  } else { 
    digitalWrite(bit1, LOW);
  }
  delayMicroseconds(clk_period/2);
  
  // cycle 5: test bit0
  digitalWrite(bit0, HIGH);
  delayMicroseconds(clk_period/2);
  if (digitalRead(v_in)){
    result += 1;
  } else { 
    digitalWrite(bit0, LOW);
  }
  delayMicroseconds(clk_period/2);
  
  // print result to serial monitor
  Serial.println(result, HEX);
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

