/* Pin definitions */
#define RED_LED_PIN_HORIZONTAL      8
#define GREEN_LED_PIN_HORIZONTAL    9
#define YELLOW_LED_PIN_HORIZONTAL   10
#define RED_LED_PIN_VERTICAL        11
#define GREEN_LED_PIN_VERTICAL      12
#define YELLOW_LED_PIN_VERTICAL     13

#define BUTTON_PIN_PEDESTRIAN       2
#define BUTTON_PIN_ADVANCED_GREEN   3
#define PEDESTRIAN_HORIZENTAL       4
#define PEDESTRIAN_VERTICAL         5
#define ADVDANCED_GREEN_HORIZENTAL  6
#define ADVDANCED_GREEN_VERTICAL    7

/* Timer macros */
#define COMPARE_MATCH_REG(f) (16000000) / (f*1024) - 1

#define TIMER_ONE_DEFAULT_FREQUENCY_HZ 1
#define TIMER_TWO_DEFAULT_FREQUENCY_HZ 1000

#define STATE_ONE_TIME_DEFAULT    11
#define STATE_TWO_TIME_DEFAULT    7
#define STATE_THREE_TIME_DEFAULT  5
#define STATE_FOUR_TIME_DEFAULT   1
#define MAX_CLOCK_TIME_DEFAULT    12

uint32_t timerSeconds = 0; // Counter to keep track of the number of seconds that have elapsed
uint64_t timerMillis = 0; // Counter to keep track of the number of milliseconds that have elapsed

uint8_t stateOneTime = STATE_ONE_TIME_DEFAULT;
uint8_t stateTwoTime = STATE_TWO_TIME_DEFAULT;
uint8_t stateThreeTime = STATE_THREE_TIME_DEFAULT;
uint8_t stateFourTime = STATE_FOUR_TIME_DEFAULT;
uint8_t maxClockTime = MAX_CLOCK_TIME_DEFAULT;

bool pedestrianButtonHorizentalPressed = false;
bool pedestrianButtonVerticalPressed = false;
bool advancedGreenHorizentalButtonPressed = false;
bool advancedGreenVerticalButtonPressed = false;

typedef enum {
  RED,
  YELLOW,
  GREEN,
  ADVANCED_GREEN
} traffic_light_states_t;

typedef enum {
  VERTICAL,
  HORIZONTAL,
} traffic_light_inputs_t;

typedef struct {
  traffic_light_states_t curLightState;
  traffic_light_inputs_t pedsInputs;
} traffic_light_t;

typedef enum {
  STATE1, // Horizontal GREEN and Vertical RED
  STATE2, // Horizontal YELLOW and Vertical RED
  STATE3, // Horizontal RED and Vertical GREEN
  STATE4, // Horizontal RED and Vertical YELLOW
} traffic_light_intersection_states_t;

typedef struct {
  traffic_light_t trafficLightHorizontal;
  traffic_light_t trafficLightVertical;
  traffic_light_intersection_states_t intersectionState;
} traffic_light_intersection_t;

void setup() {
  //set pins as outputs
  pinMode(RED_LED_PIN_HORIZONTAL, OUTPUT);
  pinMode(GREEN_LED_PIN_HORIZONTAL, OUTPUT);
  pinMode(YELLOW_LED_PIN_HORIZONTAL, OUTPUT);
  pinMode(RED_LED_PIN_VERTICAL, OUTPUT);
  pinMode(GREEN_LED_PIN_VERTICAL, OUTPUT);
  pinMode(YELLOW_LED_PIN_VERTICAL, OUTPUT);

  pinMode(BUTTON_PIN_PEDESTRIAN, INPUT);
  pinMode(BUTTON_PIN_ADVANCED_GREEN, INPUT);

  pinMode(PEDESTRIAN_HORIZENTAL, INPUT);
  pinMode(PEDESTRIAN_VERTICAL, INPUT);
  pinMode(ADVDANCED_GREEN_HORIZENTAL, INPUT);
  pinMode(ADVDANCED_GREEN_VERTICAL, INPUT);

  setTimer1Freq(TIMER_ONE_DEFAULT_FREQUENCY_HZ);

  setTimer2Freq(TIMER_TWO_DEFAULT_FREQUENCY_HZ);

  Serial.begin(9600);

  // Set the initial state

  setIntersectionState(STATE1);

  // We need two have multiple inputs per interrupt, so we we set a unique pin HIGH for each event
  // For example, hardware interrupt 1 is for both pedestrian direction. We also set pins 4 and 5 HIGH for parallel and perpendicular. In the interrupt, we check which pin is
  // HIGH to perform some action.
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN_PEDESTRIAN), pedestrianButtonInterrupt, RISING);
  // attachInterrupt(digitalPinToInterrupt(BUTTON_PIN_ADVANCED_GREEN), advancedGreenButtonInterrupt, RISING);
}

void setTimer1Freq(float freqHz){
  cli();//stop interrupts

  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  uint32_t compareMatchRegister = COMPARE_MATCH_REG(freqHz);
  OCR1A = compareMatchRegister;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei();
}

void setTimer2Freq(float freqHz)
{
  cli();//stop interrupts

  //set timer1 interrupt at 1Hz
  TCCR2A = 0;// set entire TCCR1A register to 0
  TCCR2B = 0;// same for TCCR1B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  uint32_t compareMatchRegister = COMPARE_MATCH_REG(freqHz);
  OCR2A = compareMatchRegister;// = (16*10^6) / (1000*1024) - 1 (must be <255)
  // turn on CTC mode
  TCCR2A |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR2B |= (1 << CS12) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE1A);

  sei();
}

void loop() {
  trafficLightController();
}

void pedestrianButtonInterrupt() {
  /* DO SOME STUFF HERE TO CHANGE THE TIMERS */

  if (digitalRead(PEDESTRIAN_HORIZENTAL) == HIGH)
  {
    pedestrianButtonHorizentalPressed = true;
    Serial.println("Horizental pedestrian button activated");
  }
  else if (digitalRead(PEDESTRIAN_VERTICAL) == HIGH)
  {
    pedestrianButtonVerticalPressed = true;
    Serial.println("Vertical pedestrian button activated");
  }

  Serial.println("----");
}

void advancedGreenButtonInterrupt() {
  /* DO SOME STUFF HERE TO CHANGE THE TIMERS */
  if (digitalRead(ADVDANCED_GREEN_HORIZENTAL) == HIGH)
  {
    advancedGreenHorizentalButtonPressed = true;
    Serial.println("Horizental advanced green button activated");
  }
  else if (digitalRead(ADVDANCED_GREEN_VERTICAL) == HIGH)
  {
    advancedGreenVerticalButtonPressed = true;
    Serial.println("Vertical advanced green button activated");
  }
  Serial.println("++++");
}

void setIntersectionState(traffic_light_intersection_states_t intersectionState) {
  resetLights(); /* Reset all the lights before setting them again */

  switch(intersectionState) {
    case STATE1:
      if(advancedGreenHorizentalButtonPressed == true) {
        digitalWrite(RED_LED_PIN_VERTICAL, HIGH);
        advancedGreenFlashing(GREEN_LED_PIN_HORIZONTAL);
        maxClockTime = maxClockTime + 6;
        advancedGreenHorizentalButtonPressed = false;
        Serial.println("Horizental advanced green activated");
      }
      if(pedestrianButtonHorizentalPressed == true) {
        digitalWrite(GREEN_LED_PIN_HORIZONTAL, HIGH);
        digitalWrite(RED_LED_PIN_VERTICAL, HIGH);
        delayMillis(6000);
        maxClockTime = maxClockTime + 6;
        pedestrianButtonHorizentalPressed = false;
        Serial.println("Horizental pedestrian button activated");
      }
      else
      {
        digitalWrite(GREEN_LED_PIN_HORIZONTAL, HIGH);
        digitalWrite(RED_LED_PIN_VERTICAL, HIGH);
      }
    break;

    case STATE2:
      digitalWrite(YELLOW_LED_PIN_HORIZONTAL, HIGH);
      digitalWrite(RED_LED_PIN_VERTICAL, HIGH);
    break;

    case STATE3:
      if(advancedGreenVerticalButtonPressed == true) {
        digitalWrite(RED_LED_PIN_HORIZONTAL, HIGH);
        advancedGreenFlashing(GREEN_LED_PIN_VERTICAL);
        maxClockTime = maxClockTime + 6;
        advancedGreenVerticalButtonPressed = false;
        Serial.println("Horizental advanced light activated");
      }
      if(pedestrianButtonVerticalPressed == true) {
        digitalWrite(GREEN_LED_PIN_VERTICAL, HIGH);
        digitalWrite(RED_LED_PIN_HORIZONTAL, HIGH);
        delayMillis(6000);
        //delayMillis(6);
        maxClockTime = maxClockTime + 6;
        pedestrianButtonVerticalPressed = false;
        Serial.println("Horizental pedestrian button activated");
      }
      else
      {
        digitalWrite(RED_LED_PIN_HORIZONTAL, HIGH);
        digitalWrite(GREEN_LED_PIN_VERTICAL, HIGH);
      }
    break;

    case STATE4:
      digitalWrite(RED_LED_PIN_HORIZONTAL, HIGH);
      digitalWrite(YELLOW_LED_PIN_VERTICAL, HIGH);
    break;
  }
}

void trafficLightController() {
  // Serial.println(timerSeconds);
  if(maxClockTime - stateOneTime == timerSeconds) {
    setIntersectionState(STATE1);
  }
  else if (maxClockTime - stateTwoTime == timerSeconds) {
    setIntersectionState(STATE2);
  }
  else if (maxClockTime - stateThreeTime == timerSeconds) {
    setIntersectionState(STATE3);
  }
  else if (maxClockTime - stateFourTime == timerSeconds) {
    setIntersectionState(STATE4);
  }
  else if (timerSeconds > maxClockTime) {
    resetStateTimes();
    timerSeconds = 0;
  }
}

void resetLights() {
  digitalWrite(GREEN_LED_PIN_HORIZONTAL, LOW);
  digitalWrite(YELLOW_LED_PIN_HORIZONTAL, LOW);
  digitalWrite(RED_LED_PIN_HORIZONTAL, LOW);
  digitalWrite(GREEN_LED_PIN_VERTICAL, LOW);
  digitalWrite(YELLOW_LED_PIN_VERTICAL, LOW);
  digitalWrite(RED_LED_PIN_VERTICAL, LOW);
}

void resetStateTimes() {
  stateOneTime = STATE_ONE_TIME_DEFAULT;
  stateTwoTime = STATE_TWO_TIME_DEFAULT;
  stateThreeTime = STATE_THREE_TIME_DEFAULT;
  stateFourTime = STATE_FOUR_TIME_DEFAULT;
  maxClockTime = MAX_CLOCK_TIME_DEFAULT;
}

void delayMillis(uint32_t delayValueMillis) {

  uint32_t startTime = timerMillis;

  while(timerMillis - startTime < delayValueMillis)
  {
    uint64_t countdown = timerMillis - startTime;
    Serial.println(countdown);
  }

  return;
}

void advancedGreenFlashing( uint8_t pin )
{
  uint32_t initialTime = timerSeconds;

  while (initialTime + 6 > timerSeconds)
  {
    digitalWrite(pin, HIGH);
    delayMillis(100);
    digitalWrite(pin, LOW);
    delayMillis(100);
  }
}

ISR(TIMER1_COMPA_vect){
  timerSeconds++;
  Serial.println(maxClockTime);
}

ISR(TIMER2_COMPA_vect){
  timerMillis++;
}