#include <Adafruit_MotorShield.h>

#define TdsSensorPin A1
#define VREF 5.0      // analog reference voltage(Volt) of the ADC
#define SCOUNT  30           // sum of sample point

// Create a motor shield object with defaul I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

//Create objects for each motor port
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);
Adafruit_DCMotor *motor3 = AFMS.getMotor(3);
Adafruit_DCMotor *motor4 = AFMS.getMotor(4);
int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0, temperature = 25;

//Define input pins for radio signals
int PWM_PIN1 = 3; //channel 1
int PWM_PIN2 = 5; //channel 2
int PWM_PIN3 = 6; //channel 3

void setup() {
  AFMS.begin();

  //set speed for each of the motors, then turn off
  motor1->setSpeed(75); 
  motor2->setSpeed(25);
  motor3->setSpeed(25);
  motor4->setSpeed(35);
  motor1->run(FORWARD);
  motor1->run(RELEASE);
  motor2->run(FORWARD);
  motor2->run(RELEASE);
  motor3->run(FORWARD);
  motor3->run(RELEASE);
  motor4->run(FORWARD);
  motor4->run(RELEASE);

  Serial.begin(115200);
  pinMode(TdsSensorPin, INPUT);

}

void loop() {
  //read PWM Pulse
  int pwm1 = pulseIn(PWM_PIN1, HIGH);
  int pwm2 = pulseIn(PWM_PIN2, HIGH);
  int pwm3 = pulseIn(PWM_PIN3, HIGH);
  
  
  //caudal fin control
  while (pwm1 > 1400) {
    motor1->run(FORWARD);
  pwm1 = pulseIn(PWM_PIN1, HIGH);
  pwm2 = pulseIn(PWM_PIN2, HIGH);
  pwm3 = pulseIn(PWM_PIN3, HIGH);
    sensorFunction();
  }
  //stop caudal fin motor
  motor1->run(RELEASE);
  
  //turning propeller control
  while (pwm3 > 1800) {
    motor4->run(FORWARD);
    pwm1 = pulseIn(PWM_PIN1, HIGH);
  pwm2 = pulseIn(PWM_PIN2, HIGH);
  pwm3 = pulseIn(PWM_PIN3, HIGH);
    sensorFunction();
    if (pwm1 > 1400) {
      break;
    }
  }

  while (pwm3 < 1400) {
    motor4->run(BACKWARD);
    pwm1 = pulseIn(PWM_PIN1, HIGH);
  pwm2 = pulseIn(PWM_PIN2, HIGH);
  pwm3 = pulseIn(PWM_PIN3, HIGH);
    sensorFunction();
    if (pwm1 > 1400) {
      break;
    }
  }

  motor4->run(RELEASE);

  //vertical movement propeller control
  while (pwm2 > 1800) {
    
    motor2->run(FORWARD);
    motor3->run(BACKWARD);
    pwm1 = pulseIn(PWM_PIN1, HIGH);
  pwm2 = pulseIn(PWM_PIN2, HIGH);
  pwm3 = pulseIn(PWM_PIN3, HIGH);
    sensorFunction();
    
    if (pwm1 > 1400) {
      break;
    }
    
  }

  while (pwm2 < 1400) {
    motor2->run(BACKWARD);
    motor3->run(FORWARD);
    pwm1 = pulseIn(PWM_PIN1, HIGH);
  pwm2 = pulseIn(PWM_PIN2, HIGH);
  pwm3 = pulseIn(PWM_PIN3, HIGH);
    sensorFunction();

    if (pwm1 > 1400) {
      break;
    }
  }

  //stop vertical propeller
  motor2->run(RELEASE);
  motor3->run(RELEASE);
  sensorFunction();
  
}

void sensorFunction() {
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U)  //every 40 milliseconds,read the analog value from the ADC
  {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
      analogBufferIndex = 0;
  }
  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U)
  {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVolatge = averageVoltage / compensationCoefficient; //temperature compensation
    tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5; //convert voltage value to tds value
    //Serial.print("voltage:");
    //Serial.print(averageVoltage,2);
    //Serial.print("V   ");
    Serial.print("TDS----Value:");
    Serial.print(tdsValue, 0);
    Serial.println("ppm");
  }
}

int getMedianNum(int bArray[], int iFilterLen)
{
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++)
  {
    for (i = 0; i < iFilterLen - j - 1; i++)
    {
      if (bTab[i] > bTab[i + 1])
      {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}
