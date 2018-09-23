int baudRate = 9600;   // serial connection speed
int clk_period = 100;    // simulated clock period in ms

int bit7 = 3;          // MSB, P3.4
int bit6 = 4;          // P3.3
int bit5 = 5;          // P1.6
int bit4 = 6;          // P6.6
int bit3 = 7;          // P3.2
int bit2 = 8;          // P2.7
int bit1 = 9;          // P4.2
int bit0 = 10;         // LSB, P4.1

int v_in = 2;           // P6.5

int result = 0;

void setup() { 

  // Start a serial connection to send data to the computer
  Serial.begin(baudRate);

  // Setup the digital pins as outputs 
  pinMode(bit7, OUTPUT); 
  pinMode(bit6, OUTPUT); 
  pinMode(bit5, OUTPUT); 
  pinMode(bit4, OUTPUT); 
  pinMode(bit3, OUTPUT); 
  pinMode(bit2, OUTPUT); 
  pinMode(bit1, OUTPUT); 
  pinMode(bit0, OUTPUT); 

  // Set the initial values of these pins to LOW (0V)
  digitalWrite(bit7, LOW);
  digitalWrite(bit6, LOW);
  digitalWrite(bit5, LOW);
  digitalWrite(bit4, LOW);
  digitalWrite(bit3, LOW);
  digitalWrite(bit2, LOW);
  digitalWrite(bit1, LOW);
  digitalWrite(bit0, LOW);
  
  Serial.print("Setup done\n");
  reset_blinker();
}

void loop () {  
  // cycle 1: clean all 
  digitalWrite(bit7, LOW);
  digitalWrite(bit6, LOW);
  digitalWrite(bit5, LOW);
  digitalWrite(bit4, LOW); 
  digitalWrite(bit3, LOW);
  digitalWrite(bit2, LOW);
  digitalWrite(bit1, LOW);
  digitalWrite(bit0, LOW);
  result = 0;

  // cycle 2: test bit7
  digitalWrite(bit7, HIGH);
  delayMicroseconds(clk_period/2);
  if (digitalRead(v_in)){
    result += 128;
  } else { 
    digitalWrite(bit7, LOW);
  }
  delayMicroseconds(clk_period/2);
    
  // cycle 3: test bit6
  digitalWrite(bit6, HIGH);
  delayMicroseconds(clk_period/2);
  if (digitalRead(v_in)){
    result += 64;
  } else { 
    digitalWrite(bit6, LOW);
  }
  delayMicroseconds(clk_period/2); 
    
   // cycle 4: test bit5
  digitalWrite(bit5, HIGH);
  delayMicroseconds(clk_period/2);
  if (digitalRead(v_in)){
    result += 32;
  } else { 
    digitalWrite(bit5, LOW);
  }
  delayMicroseconds(clk_period/2);

  // cycle 5: test bit4
  digitalWrite(bit4, HIGH);
  delayMicroseconds(clk_period/2);
  if (digitalRead(v_in)){
    result += 16;
  } else { 
    digitalWrite(bit4, LOW);
  }
  delayMicroseconds(clk_period/2);
  
  // cycle 6: test bit3
  digitalWrite(bit3, HIGH);
  delayMicroseconds(clk_period/2);
  if (digitalRead(v_in)){
    result += 8;
  } else { 
    digitalWrite(bit3, LOW);
  }
  delayMicroseconds(clk_period/2);
  
  // cycle 7: test bit2
  digitalWrite(bit2, HIGH);
  delayMicroseconds(clk_period/2);
  if (digitalRead(v_in)){
    result += 4;
  } else { 
    digitalWrite(bit2, LOW);
  }
  delayMicroseconds(clk_period/2);
  
  // cycle 84: test bit1
  digitalWrite(bit1, HIGH);
  delayMicroseconds(clk_period/2);
  if (digitalRead(v_in)){
    result += 2;
  } else { 
    digitalWrite(bit1, LOW);
  }
  delayMicroseconds(clk_period/2);
  
  // cycle 9: test bit0
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

