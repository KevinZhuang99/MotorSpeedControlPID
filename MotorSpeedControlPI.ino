#define ENCA 2
#define ENCB 3 // encoding pins

#define PWM 5 // ena
#define IN1 6
#define IN2 7

long prevTime = 0;
long prevPos = 0;
volatile long pos_i = 0; // encoder counts motor has moved from beginning position

float v1Filt = 0;
float v1Prev = 0;

float eintegral=0;

void setup() {
  Serial.begin(115200);

  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCB), readEncoder, RISING);
}

void loop() {



  long time = micros();

  if (time - prevTime >= 10000) { 
    // creates more stable rpm measurements by using fixed time interval

    // reads pos and velocity
    long pos = 0;

    noInterrupts();
    pos = pos_i;
    interrupts();

    // calculate velocity
    float deltaT = (float)(time - prevTime) / 1.0e6; // change in time in seconds
    float deltaPos = (pos - prevPos);
    float velocity = deltaPos / deltaT;

    prevPos = pos;
    prevTime = time;

    // convert encoder counts to rpm
    float v1 = velocity / (425) * 60;  

    // low-pass filter (25hz cutoff)
    v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
    v1Prev = v1;

    //target speed
    float target;

    if (millis() < 2000) {
      target = 0;
    } else {
      target = 100;
    }

    // pi controller
    float kp=2;
    float ki=3.5;
    float error=target-v1Filt;
    eintegral=eintegral+error*deltaT;
    float u = kp*error + ki*eintegral;
    // Derivative action was omitted because encoder noise made the response less stable.

    int dir=1;
    int pwr = (int)u;

    if (pwr>255){
      pwr=255;
    }
    if (pwr<0){
      pwr=0;
    }


    setMotor(dir,PWM,pwr,IN1,IN2);

  static unsigned long lastPrint = 0;

  if (millis() - lastPrint >= 100) {
    Serial.print(v1Filt);
    Serial.print(" ");
    Serial.println(target);

    lastPrint = millis();
  }
    delay(1);
  }
}

void setMotor(int dir, int pwm, int pwmValue, int in1, int in2) {
  analogWrite(pwm, pwmValue);

  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir == -1) { // swaps motor direction
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else { // 0 speed
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void readEncoder() {
  pos_i++;
}