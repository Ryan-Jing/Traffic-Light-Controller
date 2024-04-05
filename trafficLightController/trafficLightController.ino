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

#define STATE_ONE_DURATION    10
#define STATE_TWO_DURATION    2
#define STATE_THREE_DURATION  10
#define STATE_FOUR_DURATION   2

#define STATE_ONE_TIME_DEFAULT    0
#define STATE_TWO_TIME_DEFAULT    STATE_ONE_TIME_DEFAULT + STATE_ONE_DURATION
#define STATE_THREE_TIME_DEFAULT  STATE_TWO_TIME_DEFAULT + STATE_TWO_DURATION
#define STATE_FOUR_TIME_DEFAULT   STATE_THREE_TIME_DEFAULT + STATE_THREE_DURATION
#define MAX_CLOCK_TIME_DEFAULT    STATE_FOUR_TIME_DEFAULT + STATE_FOUR_DURATION

uint32_t timerSeconds = 0; // Counter to keep track of the number of seconds that have elapsed
uint64_t timerMillis = 0; // Counter to keep track of the number of milliseconds that have elapsed

uint64_t previousMillis = 0;
const uint8_t interval = 200;

uint8_t stateOneTime = STATE_ONE_TIME_DEFAULT;
uint8_t stateTwoTime = STATE_TWO_DURATION;
uint8_t stateThreeTime = STATE_THREE_DURATION;
uint8_t stateFourTime = STATE_FOUR_DURATION;
uint8_t maxClockTime = MAX_CLOCK_TIME_DEFAULT;

bool blinkLedVertical = false;
bool blinkLedHorizental = false;

bool pedestrianButtonPressed = false;

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

typedef enum {
  STATE1, // Horizontal GREEN and Vertical RED
  STATE2, // Horizontal YELLOW and Vertical RED
  STATE3, // Horizontal RED and Vertical GREEN
  STATE4, // Horizontal RED and Vertical YELLOW
} traffic_light_intersection_states_t;

traffic_light_intersection_states_t curIntersectionState = STATE1;

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
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN_ADVANCED_GREEN), advancedGreenButtonInterrupt, RISING);
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

  if(blinkLedHorizental)
  {
    uint64_t currentMillis = timerMillis;  // Get the current time

    if (currentMillis - previousMillis >= interval)
    {
      // Save the last time LED was updated
      previousMillis = currentMillis;

      // Toggle the LED state
      if (digitalRead(GREEN_LED_PIN_HORIZONTAL) == LOW)
      {
        digitalWrite(GREEN_LED_PIN_HORIZONTAL, HIGH);  // Turn on the LED
      }
      else
      {
        digitalWrite(GREEN_LED_PIN_HORIZONTAL, LOW);   // Turn off the LED
      }
    }
  }

  if(blinkLedVertical)
  {
    uint64_t currentMillis = timerMillis;  // Get the current time

    if (currentMillis - previousMillis >= interval)
    {
      // Save the last time LED was updated
      previousMillis = currentMillis;

      // Toggle the LED state
      if (digitalRead(GREEN_LED_PIN_VERTICAL) == LOW)
      {
        digitalWrite(GREEN_LED_PIN_VERTICAL, HIGH);  // Turn on the LED
      }
      else
      {
        digitalWrite(GREEN_LED_PIN_VERTICAL, LOW);   // Turn off the LED
      }
    }
  }

  // if(blinkLedVertical)
  // {
  //   if (timerSeconds % 2 == 0) {
  //     // If current time is even, turn on the LED
  //     digitalWrite(GREEN_LED_PIN_VERTICAL, HIGH);
  //   }
  //   else {
  //     // If current time is odd, turn off the LED
  //     digitalWrite(GREEN_LED_PIN_VERTICAL, LOW);
  //   }
  // }

  if(timerSeconds == stateOneTime + 6 && blinkLedHorizental == true)
  {
    blinkLedHorizental = false;
  }

  if(timerSeconds == stateThreeTime + 6 && blinkLedVertical == true)
  {
    blinkLedVertical = false;
  }
}

void pedestrianButtonInterrupt() {
  /* DO SOME STUFF HERE TO CHANGE THE TIMERS */
  pedestrianButtonPressed = true;
  Serial.println("Pedestrian Button Pressed");

  if (digitalRead(PEDESTRIAN_HORIZENTAL) == HIGH && pedestrianButtonHorizentalPressed == false)
  {
    if (curIntersectionState == STATE3) {
      uint32_t remainingTime = STATE_FOUR_TIME_DEFAULT - timerSeconds;

      stateFourTime -= 0.3 * (remainingTime);
      maxClockTime -= 0.3 * (remainingTime);
      pedestrianButtonHorizentalPressed = true;
      Serial.println("Pedestrian Horizental");
    }
  }
  else if (digitalRead(PEDESTRIAN_VERTICAL) == HIGH && pedestrianButtonVerticalPressed == false)
  {
    if (curIntersectionState == STATE1) {
      uint32_t remainingTime = STATE_TWO_TIME_DEFAULT - timerSeconds;

      // stateTwoTime -= 0.3 *(remainingTime);
      stateThreeTime -= 0.3 *(remainingTime);
      maxClockTime -= 0.3 *(remainingTime);
      pedestrianButtonVerticalPressed = true;
      Serial.println("Pedestrian Vertical");
    }
  }
}

void advancedGreenButtonInterrupt() {
  Serial.println("Advanced Green Button Pressed");
  /* DO SOME STUFF HERE TO CHANGE THE TIMERS */
  if (digitalRead(ADVDANCED_GREEN_HORIZENTAL) == HIGH && advancedGreenHorizentalButtonPressed == false)
  {
    advancedGreenHorizentalButtonPressed = true;
    Serial.println("Advanced Green Horizental");
  }
  else if (digitalRead(ADVDANCED_GREEN_VERTICAL) == HIGH && advancedGreenVerticalButtonPressed == false)
  {
    advancedGreenVerticalButtonPressed = true;
    Serial.println("Advanced Green Vertical");
  }
}

void setIntersectionState(traffic_light_intersection_states_t intersectionState) {
  resetLights(); /* Reset all the lights before setting them again */

  switch(intersectionState) {
    case STATE1:
      if(advancedGreenHorizentalButtonPressed == true) {
        digitalWrite(RED_LED_PIN_VERTICAL, HIGH);
        advancedGreenFlashing(GREEN_LED_PIN_HORIZONTAL, 0);
        advancedGreenHorizentalButtonPressed = false;
      }
      if(pedestrianButtonHorizentalPressed == true && pedestrianButtonPressed == true) {
        digitalWrite(GREEN_LED_PIN_HORIZONTAL, HIGH);
        digitalWrite(RED_LED_PIN_VERTICAL, HIGH);
        pedestrianButtonHorizentalPressed = false;
        pedestrianButtonPressed = false;
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
      advancedGreenFlashing(GREEN_LED_PIN_VERTICAL, 1);
      advancedGreenVerticalButtonPressed = false;
    }
    if(pedestrianButtonVerticalPressed == true) {
      digitalWrite(GREEN_LED_PIN_VERTICAL, HIGH);
      digitalWrite(RED_LED_PIN_HORIZONTAL, HIGH);
      pedestrianButtonVerticalPressed = false;
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

  if(timerSeconds == stateOneTime) {
    setIntersectionState(STATE1);
    curIntersectionState = STATE1;
  }
  else if (timerSeconds == stateTwoTime) {
    setIntersectionState(STATE2);
    curIntersectionState = STATE2;
  }
  else if (timerSeconds == stateThreeTime) {
    setIntersectionState(STATE3);
    curIntersectionState = STATE3;
  }
  else if (timerSeconds == stateFourTime) {
    setIntersectionState(STATE4);
    curIntersectionState = STATE4;
  }
  else if (timerSeconds >= maxClockTime) {
    resetStateTimes();
    timerSeconds = 0;
    timerMillis = 0;
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

void advancedGreenFlashing( uint8_t pin, uint8_t direction )
{
  uint32_t addedTime = 6;

  if( direction == 0 )
  {
    blinkLedHorizental = true;
    stateTwoTime += addedTime;
    stateThreeTime += addedTime;
    stateFourTime += addedTime;
    maxClockTime += addedTime;
  }

  else
  {
    blinkLedVertical = true;
    stateFourTime += addedTime;
    maxClockTime += addedTime;
  }
}

ISR(TIMER1_COMPA_vect){
  timerSeconds++;
  // Serial.println(maxClockTime);
}

ISR(TIMER2_COMPA_vect){
  timerMillis++;
}