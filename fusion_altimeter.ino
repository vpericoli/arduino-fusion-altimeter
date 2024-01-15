/*  
 *  Determine current vehicle altitude by fusing barometric
 *  and GPS data. Display this altitude to screen.  
 *  Fusion algorithm is based on Zaliva and Franchetti (2014).
 */ 

//
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// includes
//

//#define DEBUG

#include <Wire.h> // Needed for I2C
#include <SFE_MicroOLED.h> // OLED screen
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> // GPS
#include <SparkFun_BMP581_Arduino_Library.h> // Pressure sensor
#include <CircularBuffer.h> // Circular buffer
#include <cfloat> // for FLT_MAX

#ifdef DEBUG
  #include <malloc.h> // for stackCount()
#endif

//
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// init + globals
//

// MicroOLED Definition:
//   The library assumes a reset pin is necessary. 
//   The Qwiic OLED has RST hard-wired, so pick an arbitrarty IO pin that is not being used
#define PIN_RESET 9  
// The DC_JUMPER is the I2C Address Select jumper. 
// Set to 1 if the jumper is open (Default), or set to 0 if it's closed.
#define DC_JUMPER 1 
// OLED object with I2C declaration
MicroOLED oled(PIN_RESET, DC_JUMPER);    

// GPS and Pressure Sensor Definitions:
SFE_UBLOX_GNSS myGNSS;
BMP581 pressureSensor;

// Circular buffer size
// when setting these sizes, consider the below update rates
const int bufferSizeP = 200;
const int bufferSizeGNSS = 50;
const int arraySizeJ = bufferSizeP/5; // ensure bufferSizeP is divisible by 5

// Define circular buffers for GNSS data and associated arrays
CircularBuffer<float, bufferSizeGNSS> altitudeBuffGNSS;
CircularBuffer<float, bufferSizeGNSS> altitudeVarBuffGNSS;
CircularBuffer<unsigned long, bufferSizeGNSS> timeBuffGNSS;
float altitudeArrGNSS[bufferSizeGNSS];
float altitudeVarArrGNSS[bufferSizeGNSS];
unsigned long timeArrGNSS[bufferSizeGNSS];

// Define circular buffers for pressure data and associated arrays
CircularBuffer<float, bufferSizeP> altitudeBuffP;
CircularBuffer<unsigned long, bufferSizeP> timeBuffP;
float altitudeArrP[bufferSizeP];
unsigned long timeArrP[bufferSizeP];

// altitude related variables
int altitude = -1;
char altitudeStr[6] = "-1";   // Altitudes will be 5 or less characters. Need 6 for null-termination char.
float baro_bias = 0.0f;        // barometric bias estimate
float current_pressure = 0.0f; // current pressure reading

// Timing Variables
unsigned long lastPressureCheck = 0;
unsigned long lastUpdate = 0;
unsigned long lastBiasUpdate = 0;
const uint8_t gnssUpdateFreq = 1; // Hz, GNSS update rate
const unsigned long pressureInterval = 250;  // 4 Hz Pressure update rate (250 ms)
const unsigned long biasUpdateInterval = 1000*30; // 30 seconds
const unsigned long displayUpdateInterval = 1000; // 1 Hz display update rate

// conversion constants
static const float m2ft = 3.28084f;

// debug timing functions
#ifdef DEBUG
  unsigned long DEBUG_UPDATE = 0;
  unsigned long DEBUG_UPDATE_HEADER = 0;
#endif

//
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// arduino setup
//

void setup() {
  // boot-up time
  delay(100); 

  #ifdef DEBUG
    // Initialize serial communication
    Serial.begin(115200);
    // Wait for the serial port to connect.
    while (!Serial);
    // print stack size
    Serial.print("Free stack: ");
    Serial.println(stackCount());
    // enable GNSS debugging output messages
    myGNSS.enableDebugging(Serial, true); //Enable only the critical debug messages over Serial
  #endif 

  // initialize I2C
  Wire.begin();

  // Initialize the screen
  oled.begin();
  oled.clear(ALL);
  oled.display();  // Display what's in the buffer (splashscreen)
  delay(1000); 
  oled.clear(PAGE);
  print_oled("Init...", 0);

  // Initialize GNSS module
  if( myGNSS.begin() == false) {
    // do something to alert the user that GPS module connection failed
    print_oled("GNSS failed init.", 0);
    #ifdef DEBUG
      Serial.println("GNSS failed init.");
    #endif
    while (1);
  }
  
  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.setNavigationFrequency(gnssUpdateFreq); // integer Hz. 
  myGNSS.setAutoPVT(true); // in this mode, getPVT() only operates when new data is available without blocking
  myGNSS.setDynamicModel(DYN_MODEL_AUTOMOTIVE); // Set the dynamic model to automotive
  

  // Initialize BMP581 pressure sensor
  if( pressureSensor.beginI2C(BMP581_I2C_ADDRESS_DEFAULT) != BMP5_OK ) {
    // do something to alert the user that BMP module connection failed
    print_oled("BMP581 failed init.", 0);
    #ifdef DEBUG
      Serial.println("BMP581 failed init.");
    #endif
    while (1);
  }

  // set the pressure sensor configuration to enable filtering...
  bmp5_iir_config config =
  {
      .set_iir_t = BMP5_IIR_FILTER_COEFF_127, // Set filter coefficient
      .set_iir_p = BMP5_IIR_FILTER_COEFF_127, // Set filter coefficient
      .shdw_set_iir_t = BMP5_ENABLE, // Store filtered data in data registers
      .shdw_set_iir_p = BMP5_ENABLE, // Store filtered data in data registers
      .iir_flush_forced_en = BMP5_DISABLE // Flush filter in forced mode
  };
  pressureSensor.setFilterConfig(&config);
}


//
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// arduino loop
//
void loop() {
  unsigned long currentMillis = millis();

  // Check GNSS data at requested polling interval
  // When setAutoPVT(true):
  //  * The GNSS module is configured to automatically "send" data at setNavigationFrequency().
  //  * getPVT() checks if new data has been received since the last time it was called. 
  //    It does not actively request data from the GNSS module.
  //    If new data is available, getPVT() processes and returns true.
  //    If no new data is available, getPVT() returns false.
  //
  if( myGNSS.getPVT() && !myGNSS.getInvalidLlh() ) {
    // capture GNSS data if new data is available and LLH is valid 

    // pull data from GNSS board
    // getAltitude() returns altitude in mm above the WGS84 ellipsoid standard reference.
    // let's use getAltitudeMSL() instead, which returns altitude in mm above mean sea level.
    long altitudeGNSS = myGNSS.getAltitudeMSL();
    //long altitudeAccGNSS = myGNSS.getVerticalAccuracy(); // wow, that was extremely confusing to debug
    long altitudeAccGNSS = myGNSS.getVerticalAccEst(); // mm
    int SIV = myGNSS.getSIV(); // sats in view

    // push data to circular buffers, adding to the front of the buffer
    if( SIV > 2 ) {
      altitudeBuffGNSS.unshift(altitudeGNSS / 1000); // meters
      altitudeVarBuffGNSS.unshift((altitudeAccGNSS / 1000) * (altitudeAccGNSS / 1000)); // variance, meters^2
      timeBuffGNSS.unshift(currentMillis); // ms
    }
  }

  // Check BMP581 data at requested polling interval
  if (currentMillis - lastPressureCheck >= pressureInterval) {
    lastPressureCheck = currentMillis;

    bmp5_sensor_data bmp5_data = {0,0};
    int8_t err = pressureSensor.getSensorData(&bmp5_data);
    if(err == BMP5_OK) {
      // compute altitude from pressure
      current_pressure = bmp5_data.pressure; // Pa
      float altitudeP = altitude_from_pressure(current_pressure); // meters

      // push to buffers, add data to the front of the buffer
      altitudeBuffP.unshift(altitudeP); // meters
      timeBuffP.unshift(currentMillis); // ms
    }
  }

  // Update barometric bias estimate at requested interval.
  // only update if there are sufficient data in the buffers.
  // it would also be nice to update once shortly after start-up instead of having to wait for biasUpdateInterval.
  // a lot of these variables are globals, so I don't need to pass them as args. doing so anyway. 
  bool sufficient_buffer = (altitudeBuffP.size() >= 20) && (altitudeBuffGNSS.size() > 5);

  if( (currentMillis - lastBiasUpdate >= biasUpdateInterval) && sufficient_buffer ) {
    
    lastBiasUpdate = currentMillis;
    baro_bias = compute_baro_bias(altitudeBuffGNSS, altitudeVarBuffGNSS, 
                                  altitudeBuffP, timeBuffGNSS, timeBuffP, 
                                  current_pressure);
  }


  // Using barometric bias, update altitude estimate at requested interval and display to screen. 
  // Since baro_bias is initialized to zero, this will display barometric altitude until the first update.
  // If GPS cannot find a lock, this will continue to display barometric altitude as a failsafe.
  if( (currentMillis - lastUpdate >= displayUpdateInterval) && (altitudeBuffP.size() > 0) ) {

    // update the altitude estimate
    lastUpdate = currentMillis;
    altitude = altitudeBuffP[0] + baro_bias; // meters

    // convert to feet, round to nearest 10 feet
    altitude = round(altitude * m2ft / 10.0f) * 10.0f;

    // sanity check
    // It would be unreasonable to have less than -300 ft (Death Valley)
    // or more than 14300 ft (Colorado Scenic Byway) 
    if( (altitude < -300) || (altitude > 14200) ) {
      altitude = -1;
    }
    
    // display the altitude estimate
    // Altitudes will be 5 or less characters, ensured by above sanity check. 
    // Need length 6 for null-termination char.
    //sprintf(altitudeStr, "%d", static_cast<int>(altitude));
    snprintf(altitudeStr, sizeof(altitudeStr), "%d", static_cast<int>(altitude));
    print_oled(altitudeStr, 0);
  }

  // debug block
  #ifdef DEBUG
    if( currentMillis - DEBUG_UPDATE_HEADER >= 8000 ) {
      DEBUG_UPDATE_HEADER = currentMillis;
      Serial.println("GNSS size, P size, alt [ft], P alt [ft], GNSS alt [ft], GNSS std [ft], baro bias [ft]");
    }
    if( currentMillis - DEBUG_UPDATE >= 1000 ) {
      DEBUG_UPDATE = currentMillis;
      
      // arduino has issues with floats and snprintf...
      Serial.print(altitudeBuffGNSS.size());
      Serial.print(", ");
      Serial.print(altitudeBuffP.size());
      Serial.print(", ");
      Serial.print(altitude);
      Serial.print(", ");
      Serial.print(static_cast<int>(altitudeBuffP[0] * m2ft));
      Serial.print(", ");
      Serial.print(static_cast<int>(altitudeBuffGNSS[0] * m2ft));
      Serial.print(", ");
      Serial.print(static_cast<int>(altitudeVarBuffGNSS[0] * m2ft * m2ft));
      Serial.print(", ");
      Serial.println(static_cast<int>(baro_bias * m2ft));
    }
  #endif
}

//
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// function to print to screen
//

void print_oled(char* s, int font)
{
  oled.clear(PAGE);
  oled.setFontType(0);
  oled.setCursor(0,0);
  oled.print(s);
  oled.display();
}

//
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// function to estimate altitude from pressure
//

float altitude_from_pressure(float pressure) {
  // Barometric formula for the Troposphere:
  //  Pb = 101325 Pa, the average sea-level pressure
  //  Tb = 288.15 K, the reference temperature at sea level in Kelvins
  //  Lb = -0.0065 K/m, the temperature lapse rate
  //  R = 8.31447 J/(mol·K), the universal gas constant
  //  g = 9.80665 m/s2, the standard gravity
  //  M = 0.0289644 kg/mol, the molar mass of dry air
  //
  //  h = (Tb/Lb)*{[P/Pb]^[-(R*Lb)/(g*M) - 1]}
  // 
  // simplifying:
  //  h = C1 * ((P/Pb)^C2 - 1)
  //  h = C1 * (pow(ratio, C2) - 1)

  const float C1 = 288.15 / -0.0065;
  const float C2 = -1.0 * 8.31447 * -0.0065 / (9.80665 * 0.0289644); 
  const float Pb = 101325.0; // Pa
  return C1 * (pow(pressure / Pb, C2) - 1.0);
}

//
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// function to perform linear least squares and evaluate best fit at each point
//
void LLSQ_eval(const unsigned long* x, const float* y, int n, float &slope, float* yFit) {
  // Example usage:
  // unsigned long x[] = {1, 2, 3, 4, 5};
  // float y[] = {2, 3, 4, 6, 5};
  // int n = sizeof(x) / sizeof(x[0]);
  //
  // float yFit[n];
  // float slope;
  // linearLeastSquaresAndEvaluate(x, y, n, slope, yFit);
  //

  unsigned long long sumX = 0; // will this overflow?
  float sumY = 0, sumXY = 0, sumX2 = 0;
  for (int i = 0; i < n; i++) {
    sumX += x[i];
    sumY += y[i];
    sumXY += x[i] * y[i];
    sumX2 += x[i] * x[i];
  }

  float denominator = n * sumX2 - sumX * sumX;
  slope = (n * sumXY - sumX * sumY) / denominator;
  float b = (sumY * sumX2 - sumX * sumXY) / denominator;

  // Evaluate the line at each x point
  for (int i = 0; i < n; i++) {
    yFit[i] = slope * x[i] + b;
  }
}

//
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// barometric and GPS altitude sensor fusion
// ref:  V. Zaliva and F. Franchetti, "Barometric and GPS altitude sensor fusion," 
//       2014 IEEE International Conference on Acoustics, Speech and Signal Processing (ICASSP), 
//       Florence, Italy, 2014, pp. 7525-7529, doi: 10.1109/ICASSP.2014.6855063.
//

float compute_baro_bias(CircularBuffer<float, bufferSizeGNSS>& altitudeBuffGNSS, 
                        CircularBuffer<float, bufferSizeGNSS>& altitudeVarBuffGNSS, 
                        CircularBuffer<float, bufferSizeP>& altitudeBuffP,
                        CircularBuffer<unsigned long, bufferSizeGNSS>& timeBuffGNSS,
                        CircularBuffer<unsigned long, bufferSizeP>& timeBuffP,
                        float current_pressure) {

  // initialize variables
  // according to the Ref, compute J for m=10 to m=bufferSizeP.
  // to save memory and time, only compute for m divisible by 5. 
  // this means that J is computed for m=10, 15, 20, 25, ..., bufferSizeP
  // REMINDER: m and n are the actual number of data points, not indices.
  float J_buff[arraySizeJ];
  int J_buff_indx = 0;
  int m_max = altitudeBuffP.size();
  int n_max = altitudeBuffGNSS.size();
  int associated_n[arraySizeJ]; // associated n for each m
  int m;
  int n;

  // cast circular buffers to arrays... 
  // I'm assuming this is more efficient than accessing the circular buffer directly
  // but will add to memory usage...
  altitudeBuffGNSS.copyToArray(altitudeArrGNSS);
  altitudeVarBuffGNSS.copyToArray(altitudeVarArrGNSS);
  timeBuffGNSS.copyToArray(timeArrGNSS);
  altitudeBuffP.copyToArray(altitudeArrP);
  timeBuffP.copyToArray(timeArrP);

  // it could be possible for all collected GPS data to be older than all collected barometric data.
  // i.e. the most recent GNSS time is older than the oldest baro time. 
  // this would result in n = 0 for any choice of m. 
  // in this case, do not update the baro_bias. 
  if( timeArrGNSS[0] < timeArrP[m_max-1] ) {
    return baro_bias;
  }

  // loop through candidates for m, computing J for each candidate.
  // larger values of m correspond to more considered pressure data points.
  // smaller m uses more recent data, while larger m expands the time window to consider older data points.
  for( m = 10; m <= m_max; m += 5 ) {
    
    J_buff_indx = (m/5)-2; // index of J_buff corresponding to this m

    // determine the n associated with this m
    // n represents the number of GPS data points to consider.
    // but, n is not independent of m.
    // n should represent the number of GPS data points that are within the time window of m.
    // with small values of m, we may not have any GPS data points within the time window.
    // as m expands to consider older data, it may eventually encompass all captured GPS data points.
    // recall that the time is time elapsed. 
    // So, the most recent values will have larger times, while older values will have smaller times.
    n = 0;
    for( int i = 0; i < n_max; i++ ) {
      if( timeArrGNSS[i] >= timeArrP[m-1] ) {
        n = n + 1;
      }
      else {
          break;
      }
    }

    if( n == 0 ) {
      // it's possible that there are no GPS measurements within this time interval, i.e. n = 0.
      // set J to a large value so that it is not selected as the minimum.
      J_buff[J_buff_indx] = FLT_MAX;
      continue;
    }
    else {
      // otherwise, store the associated n.
      associated_n[J_buff_indx] = n; // we'll need this later.
    }

    // compute mean altitude for each sensor, over the m time window
    float meanAltitudeGNSS = arr_mean(altitudeArrGNSS, n);
    float meanAltitudeP = arr_mean(altitudeArrP, m);

    // compute GNSS altitude variance over the m time window
    float varGNSS = arr_mean(altitudeVarArrGNSS, n);

    // perform LSQ fit to barometric data
    float betaP;
    float fitP[m];
    LLSQ_eval(timeArrP, altitudeArrP, m, betaP, fitP);

    // use this to compute the residual errors of barometric data
    float eps[m];
    for( int i = 0; i < m; i++ ) {
      eps[i] = altitudeArrP[i] - fitP[i];
    }

    // obtain mean, variance, stdev, on barometric fit and residuals.
    // needed to estimate barometric sensor variance. 
    // Refer to Reference eqs 5, 6, 10, 11
    // avg beta (eq. 5):
    float avg_betaP = betaP;
    
    // var beta (eq. 6):
    // we probably don't need time to be accurate to within decimal milliseconds...
    // so, use the unsigned long mean and discard the fractional portion.
    float var_betaP = 0;
    unsigned long mean_timeArrP = arr_mean_ul(timeArrP, m);

    for (int i = 0; i < m; i++) {
      // diff promotes the unsigned longs to floats
      float diff = timeArrP[i] - mean_timeArrP;
      var_betaP += diff * diff;
    }

    var_betaP = (arr_sum(eps, m) / (m - 2)) / (var_betaP / m);

    // avg eps (eq. 10):
    float avg_eps = arr_mean(eps, m);
    
    // stdev eps (eq. 11)
    float stdev_eps = 0;
    for (int i = 0; i < m; i++) {
      float diff = eps[i] - avg_eps;
      stdev_eps += diff * diff;
    }
    stdev_eps = sqrt(stdev_eps / m);

    // barometric sensor variance (eq. 14):
    float varP = avg_betaP*avg_betaP * var_betaP * (avg_eps*avg_eps + stdev_eps) / (avg_betaP*avg_betaP + 1) \
               + stdev_eps*pow(2.0*(pow(avg_betaP,3) + avg_betaP) + var_betaP, 2) / (4.0*pow(avg_betaP*avg_betaP + 1, 3));

    // calculate the assumed maximum potential natural drift of barometric altitude (eq. 16)
    float durP = (timeArrP[0] - timeArrP[m-1]) / 1000; // duration of time window considered, in seconds 
    float driftP =  durP * 40000.0 / 3600.0; // assumes 400 hPa per hour drift rate
    float driftAlt = max( altitude_from_pressure(current_pressure - driftP) - altitudeArrP[0], 
                          altitudeArrP[0] - altitude_from_pressure(current_pressure + driftP) );

    // cost function (eq. 17)
    J_buff[J_buff_indx] = sqrt( varP + varP/m + varGNSS/n ) + 0.5*driftAlt;
  }

  // we have now computed J (cost function) for all candidates of m.
  // find the m associated with min(J)
  // J_buff is only filled up to index of: current value of J_buff_indx.
  // note that m_max may not be divisible by 5 without remainders.
  int J_min_indx = arr_min_indx(J_buff, J_buff_indx+1);
  m = (J_min_indx + 2) * 5;
  n = associated_n[J_min_indx];


  // evaluate altitude per Eq. 4.
  // Eq. 4 looks right, but the paragraph before has a typo. 
  // We already compute some of these quantities in the loop.
  // Is it better to store or to recompute?
  // this whole implementation feels like a computationally intense memory hog.
  float meanAltitudeGNSS = arr_mean(altitudeArrGNSS, n);
  float meanAltitudeP = arr_mean(altitudeArrP, m);
  return (meanAltitudeGNSS - meanAltitudeP);
}


//
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// array supporting functions
//

float arr_sum(const float* array, int size) {
    float sum = 0;
    for( int i = 0; i < size; i++ ) {
        sum += array[i];
    }
    return sum;
}

float arr_mean(const float* array, int size) {
    return arr_sum(array, size) / size;
}

unsigned long arr_mean_ul(const unsigned long* array, int size) {
    // unsigned long version.
    // simply discards the fractional portion. 
    // is long long too large? I don't want sum to overflow.
    // could alternatively do division each iteration? would round each iteration.
    unsigned long long sum = 0;
    for( int i = 0; i < size; i++ ) {
        sum += array[i];
    }
    return sum / size;
}

int arr_min_indx(const float* array, int size) {
    int min_indx = 0;
    for( int i = 1; i < size; i++ ) {
        if( array[i] < array[min_indx] ) {
            min_indx = i;
        }
    }
    return min_indx;
}

//
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// debug functions
//
#ifdef DEBUG
  extern "C" char *sbrk(int i);
  char stack_dummy = 0;

  size_t stackCount() {
    char * heapend=sbrk(0);
    register char * stack_ptr asm ("sp");
    char * current_heapend = sbrk(0);
    size_t free;

    if (stack_ptr < &stack_dummy) return 0;
    free = stack_ptr - current_heapend;
    return free;
  }

  void debug_arr_print(const float* array, int size) {
    for( int i = 0; i < size; i++ ) {
        Serial.print(array[i]);
        Serial.print(", ");
    }
    Serial.println();
  }

  void debug_arr_print_int(const int* array, int size) {
    for( int i = 0; i < size; i++ ) {
        Serial.print(array[i]);
        Serial.print(", ");
    }
    Serial.println();
  }
#endif