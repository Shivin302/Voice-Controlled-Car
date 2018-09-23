/*
 * classify.ino
 *
 * EE16B Spring 2016
 * Emily Naviasky & Nathaniel Mailoa
 *
 * EE 16B Fall 2017
 * Andrew Blatner
 *
 */

#define MIC_INPUT                   P6_0

#define SIZE                        2752
#define SIZE_AFTER_FILTER           172
#define ADC_TIMER_MS                0.35

/*---------------------------*/
/*      CODE BLOCK PCA1      */
/*---------------------------*/

// Enveloping and K-means constants
#define SNIPPET_SIZE                90
#define PRELENGTH                   7
#define THRESHOLD                   0.7

#define KMEANS_THRESHOLD            0.04
#define LOUDNESS_THRESHOLD          500

/*---------------------------*/
/*      CODE BLOCK PCA2      */
/*---------------------------*/

float pca_vec1[SNIPPET_SIZE] = {0.00899097, 0.00561217, 0.00363569, -0.01055335, -0.03906339, -0.08494901, -0.11677893, -0.17074685, -0.23094303, -0.22389627, -0.18797845, -0.19751628, -0.19864463, -0.2037736, -0.23197992, -0.21958967, -0.23030794, -0.22210908, -0.20787395, -0.20209397, -0.19792209, -0.18535966, -0.16675083, -0.13125216, -0.0789998, -0.05117317, -0.01056394, -0.0041902, 0.01621207, 0.02613218, 0.04960792, 0.04455742, 0.04849294, 0.05569485, 0.05575957, 0.06457147, 0.06117554, 0.06863627, 0.07185855, 0.08190224, 0.0868796, 0.0899904, 0.08863602, 0.10542675, 0.10933931, 0.11563764, 0.10444944, 0.10235584, 0.09398261, 0.10220856, 0.09553134, 0.08864683, 0.09719604, 0.08813342, 0.07518341, 0.07376554, 0.0754387, 0.07175801, 0.07057643, 0.06813704, 0.07490745, 0.07358203, 0.06286608, 0.05965842, 0.06239706, 0.06038663, 0.05744446, 0.06125558, 0.06259381, 0.06667366, 0.06484156, 0.05367432, 0.05249944, 0.0443795, 0.04728647, 0.03889418, 0.03376508, 0.03906491, 0.03396555, 0.03115172, 0.02533223, 0.02451458, 0.02876485, 0.02962499, 0.02607741, 0.02370335, 0.02918948, 0.02717826, 0.02633335, 0.01689096};
float pca_vec2[SNIPPET_SIZE] = {0.04416722, 0.04658042, 0.02368366, -0.02558988, -0.08404109, -0.15709971, -0.16566311, -0.18832668, -0.27057825, -0.25299553, -0.19283048, -0.19606703, -0.14709146, -0.10125801, -0.0319256, 0.03722647, 0.06321843, 0.08837951, 0.13065594, 0.11925212, 0.13117801, 0.14140614, 0.15758746, 0.20643801, 0.2329083, 0.24008372, 0.23955257, 0.22604845, 0.20542644, 0.18001935, 0.1439786, 0.10917527, 0.09972239, 0.0536061, 0.04131611, 0.00048674, -0.02605532, -0.0334954, -0.06578363, -0.08210114, -0.09237265, -0.09574675, -0.08279865, -0.11737998, -0.12857035, -0.12594718, -0.12868758, -0.12069649, -0.0999586, -0.10348694, -0.05526833, -0.07098231, -0.04980424, -0.01813032, 0.01743736, 0.03177268, 0.0372783, 0.04985884, 0.05409954, 0.04784124, 0.03174437, 0.03345447, 0.02590114, 0.0100876, 0.01177758, 0.01154714, 0.00112617, -0.00364989, 0.00677636, -0.00733611, 0.00140337, 0.00787247, 0.01165214, 0.01073424, 0.00856337, 0.00721931, 0.0208448, 0.01751397, -0.00246993, 0.01305358, -0.0056085, 0.00276428, -0.00973117, -0.01156698, -0.01263978, -0.01554517, -0.01455589, -0.01010648, -0.0068225, -0.02365662};
float mean_vec[SNIPPET_SIZE] = {0.0053532, 0.00572896, 0.00601331, 0.00792492, 0.01073427, 0.01415807, 0.01741638, 0.02398926, 0.02564078, 0.02525986, 0.02485079, 0.02448398, 0.02332551, 0.02210676, 0.02070058, 0.01921335, 0.01857841, 0.0176469, 0.01693594, 0.0161855, 0.01556385, 0.01497832, 0.01447814, 0.0136406, 0.01259774, 0.01184542, 0.01104993, 0.01092337, 0.01078859, 0.01003128, 0.00948302, 0.00948225, 0.00984921, 0.00902584, 0.00888336, 0.00863901, 0.00866091, 0.00854405, 0.00912329, 0.00899948, 0.0094364, 0.00983219, 0.00990984, 0.01054062, 0.01073293, 0.01092398, 0.01073248, 0.01044187, 0.0106949, 0.01116267, 0.01154799, 0.01233112, 0.01233716, 0.01226055, 0.01179762, 0.01152569, 0.01138554, 0.01110376, 0.01114774, 0.01099695, 0.01083192, 0.01022784, 0.00973291, 0.00936896, 0.00847534, 0.00861621, 0.00821308, 0.00835212, 0.00836312, 0.00823973, 0.00801882, 0.00770311, 0.00727656, 0.00697387, 0.0067466, 0.00659461, 0.0063042, 0.0062466, 0.00580243, 0.00608395, 0.00610481, 0.00574831, 0.00603445, 0.00601463, 0.00579419, 0.00544192, 0.00592797, 0.00595175, 0.00555791, 0.00557566};
float centroid1[2] = {0.03197428,  0.01439462};
float centroid2[2] = {-0.00725541,  0.02211973};
float centroid3[2] = {-0.06905446, -0.01142818};
float centroid4[2] = {0.0461015,  -0.03072628};
float* centroids[4] = {
  (float *) &centroid1, (float *) &centroid2,
  (float *) &centroid3, (float *) &centroid4
};

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

float result[SNIPPET_SIZE] = {0};
float proj1 = 0;
float proj2 = 0;

// Data array and index pointer
int re[SIZE] = {0};
volatile int re_pointer = 0;

/*---------------------------*/
/*       Norm functions      */
/*---------------------------*/

// Compute the L2 norm of (dim1, dim2) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        centroid: size-2 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm(float dim1, float dim2, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2));
}

// Compute the L2 norm of (dim1, dim2, dim3) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        dim3: 3rd dimension coordinate
//        centroid: size-3 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm3(float dim1, float dim2, float dim3, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2) + pow(dim3-centroid[2],2));
}

void setup(void) {
  Serial.begin(38400);

  pinMode(MIC_INPUT, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  re_pointer = 0;
  reset_blinker();
  setTimer();
}

void loop(void) {
  if (re_pointer == SIZE) {
    digitalWrite(RED_LED, LOW);

    // Apply enveloping function and get snippet with speech.
    // Do classification only if loud enough.
    if (envelope(re, result)) {

      // Reset projection result variables declared above
      proj1 = 0;
      proj2 = 0;

      /*---------------------------*/
      /*      CODE BLOCK PCA3      */
      /*---------------------------*/

      // Perform principal component projection
      // YOUR CODE HERE
      for (int i=0; i<SNIPPET_SIZE; i++){
        result[i] = result[i] - mean_vec[i];
      }
      float proj1 = 0;
      for (int i=0; i<SNIPPET_SIZE; i++){
        proj1 += result[i] * pca_vec1[i];
      }
      float proj2 = 0;
      for (int i=0; i<SNIPPET_SIZE; i++){
        proj2 += result[i] * pca_vec2[i];
      }

      // Classification - Hint: use the function l2_norm defined above
      float c1_dist = l2_norm(proj1,proj2,centroids[0]);
      float c2_dist = l2_norm(proj1,proj2,centroids[1]);
      float c3_dist = l2_norm(proj1,proj2,centroids[2]);
      float c4_dist = l2_norm(proj1,proj2,centroids[3]);
      float d1_2 = min(c1_dist,c2_dist);
      float d3_4 = min(c3_dist,c4_dist);
      float smallest = min(d1_2,d3_4);
      

      
      // Check against KMEANS_THRESHOLD and print result over serial
      // YOUR CODE HERE
      if (smallest <= KMEANS_THRESHOLD) {
      if (smallest == c1_dist)
      {
        Serial.println("Illuminati");
      }
      else if (smallest == c2_dist)
      {
        Serial.println("Feedback");
      }
      else if (smallest == c3_dist)
      {
        Serial.println("Trump");
      }
      else 
      {
        Serial.println("Technology");
      }
      }


      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    }

    delay(2000);
    re_pointer = 0;
  }
}

// Enveloping function with thresholding and normalizing,
// returns snippet of interest (containing speech)
bool envelope(int* data, float* data_out) {
  int32_t avg = 0;
  float maximum = 0;
  int32_t total = 0;
  int block;

  // Apply enveloping filter while finding maximum value
  for (block = 0; block < SIZE_AFTER_FILTER; block++) {
    avg = 0;
    for (int i = 0; i < 16; i++) {
      avg += data[i+block*16];
    }
    avg = avg >> 4;
    data[block] = abs(data[block*16] - avg);
    for (int i = 1; i < 16; i++) {
      data[block] += abs(data[i+block*16] - avg);
    }
    if (data[block] > maximum) {
      maximum = data[block];
    }
  }

  // If not loud enough, return false
  if (maximum < LOUDNESS_THRESHOLD) {
    return false;
  }

  // Determine threshold
  float thres = THRESHOLD * maximum;

  // Figure out when interesting snippet starts and write to data_out
  block = PRELENGTH;
  while (data[block++] < thres);
  if (block > SIZE_AFTER_FILTER - SNIPPET_SIZE) {
    block = SIZE_AFTER_FILTER - SNIPPET_SIZE;
  }
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data[block-PRELENGTH+i];
    total += data_out[i];
  }

  // Normalize data_out
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data_out[i] / total;
  }

  return true;
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void reset_blinker(void) {
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

/*---------------------------*/
/*    Interrupt functions    */
/*---------------------------*/

// ISR for timestep
#pragma vector=TIMER2_A0_VECTOR    // Timer A ISR
__interrupt void Timer2_A0_ISR(void) {
  if (re_pointer < SIZE) {
    digitalWrite(RED_LED, HIGH);
    re[re_pointer] = (analogRead(MIC_INPUT) >> 4) - 128;
    re_pointer += 1;
  }
}

// Set timer for timestep; use A2 since A0 & A1 are used by PWM
void setTimer(void) {
  // Set the timer based on 25MHz clock
  TA2CCR0 = (unsigned int) (25000*ADC_TIMER_MS);
  TA2CCTL0 = CCIE;
  __bis_SR_register(GIE);
  TA2CTL = TASSEL_2 + MC_1 + TACLR + ID_0;
}
