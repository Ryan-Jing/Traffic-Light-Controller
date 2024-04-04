/* Pin definitions */
#define RED_LED_PIN_HORIZONTAL      8
#define GREEN_LED_PIN_HORIZONTAL    9
#define YELLOW_LED_PIN_HORIZONTAL   10
#define RED_LED_PIN_VERTICAL        11
#define GREEN_LED_PIN_VERTICAL      12
#define YELLOW_LED_PIN_VERTICAL     13

#define BUTTON_PIN_PEDESTRIAN       2
#define BUTTON_PIN_ADVANCED_GREEN   3
#define PEDESTRIAN_HORIZONTAL       4
#define PEDESTRIAN_VERTICAL         5
#define ADVANCED_GREEN_HORIZONTAL   6
#define ADVDANCED_GREEN_VERTICAL    7

/* Timer macros */
#define COMPARE_MATCH_REG(f) (16000000) / (f*1024) - 1

#define TIMER_ONE_DEFAULT_FREQUENCY_HZ 1
#define TIMER_TWO_DEFAULT_FREQUENCY_HZ 1000

#define STATE_ONE_DURATION 10
#define STATE_TWO_DURATION 2
#define STATE_THREE_DURATION 10
#define STATE_FOUR_DURATION 2
#define ADVANCED_GREEN_DURATION_MS 2000

#define STATE_ONE_TIME_DEFAULT    0
#define STATE_TWO_TIME_DEFAULT    STATE_ONE_TIME_DEFAULT + STATE_ONE_DURATION
#define STATE_THREE_TIME_DEFAULT  STATE_TWO_TIME_DEFAULT + STATE_TWO_DURATION
#define STATE_FOUR_TIME_DEFAULT   STATE_THREE_TIME_DEFAULT + STATE_THREE_DURATION
#define MAX_CLOCK_TIME_DEFAULT    STATE_FOUR_TIME_DEFAULT + STATE_FOUR_DURATION

uint32_t timerSeconds = 0; // Counter to keep track of the number of seconds that have elapsed
int timerMillis = 0; // Counter to keep track of the number of milliseconds that have elapsed

uint8_t stateOneTime = STATE_ONE_TIME_DEFAULT;
uint8_t stateTwoTime = STATE_TWO_DURATION;
uint8_t stateThreeTime = STATE_THREE_DURATION;
uint8_t stateFourTime = STATE_FOUR_DURATION;
uint8_t maxClockTime = MAX_CLOCK_TIME_DEFAULT;

bool pedestrianButtonPressed = false;
bool pedestrianButtonHorizontalPressed = false;
bool pedestrianButtonVerticalPressed = false;
bool advancedGreenHorizontalButtonPressed = false;
bool advancedGreenVerticalButtonPressed = false;

unsigned long previousMillis = 0; // Store the last time LED was updated
const long interval = 200;        // Blink interval in milliseconds
bool blinkState = LOW;            // Initialize LED state for blinking
int startHorizontalLightBlinkTime = 0;
int startVerticalLightBlinkTime = 0;


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

  pinMode(PEDESTRIAN_HORIZONTAL, INPUT);
  pinMode(PEDESTRIAN_VERTICAL, INPUT);
  pinMode(ADVANCED_GREEN_HORIZONTAL, INPUT);
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
}

void pedestrianButtonInterrupt() {
  pedestrianButtonPressed = true;

  if (digitalRead(PEDESTRIAN_HORIZONTAL) == HIGH && pedestrianButtonHorizontalPressed == false)
  {
    pedestrianButtonHorizontalPressed = true;
    if (curIntersectionState == STATE3) {
      uint32_t remainingTime = STATE_FOUR_TIME_DEFAULT - timerSeconds;

      stateFourTime -= 0.3 * (remainingTime);
      maxClockTime -= 0.3 * (remainingTime);
    }
  }
  else if (digitalRead(PEDESTRIAN_VERTICAL) == HIGH && pedestrianButtonVerticalPressed == false)
  {
    pedestrianButtonVerticalPressed = true;
    if (curIntersectionState == STATE1) {
      uint32_t remainingTime = STATE_TWO_TIME_DEFAULT - timerSeconds;

      stateTwoTime -= 0.3 *(remainingTime);
      stateThreeTime -= 0.3 *(remainingTime);
      maxClockTime -= 0.3 *(remainingTime);
    }
  }
}

void advancedGreenButtonInterrupt() {
  /* DO SOME STUFF HERE TO CHANGE THE TIMERS */
  if (digitalRead(ADVANCED_GREEN_HORIZONTAL) == HIGH)
  {
    advancedGreenHorizontalButtonPressed = true;
    startHorizontalLightBlinkTime = timerMillis;

  }
  else if (digitalRead(ADVDANCED_GREEN_VERTICAL) == HIGH)
  {
    advancedGreenVerticalButtonPressed = true;
    startVerticalLightBlinkTime = timerMillis;
  }
}

void setIntersectionState(traffic_light_intersection_states_t intersectionState) {
  resetLights(); /* Reset all the lights before setting them again */

  switch(intersectionState) {
    case STATE1:
      if(advancedGreenHorizontalButtonPressed == true) {
        advancedGreenFlashing(GREEN_LED_PIN_HORIZONTAL, RED_LED_PIN_VERTICAL);
        if (timerMillis - startHorizontalLightBlinkTime >= ADVANCED_GREEN_DURATION_MS) {
          startHorizontalLightBlinkTime = 0;
          advancedGreenHorizontalButtonPressed = false;
        }
      }
      if(pedestrianButtonHorizontalPressed == true && pedestrianButtonPressed == true) {
        digitalWrite(GREEN_LED_PIN_HORIZONTAL, HIGH);
        digitalWrite(RED_LED_PIN_VERTICAL, HIGH);
        pedestrianButtonHorizontalPressed = false;
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
      advancedGreenFlashing(GREEN_LED_PIN_VERTICAL, RED_LED_PIN_HORIZONTAL);        
      if (timerMillis - startVerticalLightBlinkTime >= ADVANCED_GREEN_DURATION_MS) {
        startVerticalLightBlinkTime = 0;
        advancedGreenVerticalButtonPressed = false;
      }
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

  if(timerSeconds >= stateOneTime && timerSeconds < stateTwoTime) {
    setIntersectionState(STATE1);
    curIntersectionState = STATE1;
  }
  else if (timerSeconds >= stateTwoTime && timerSeconds < stateThreeTime) {
    setIntersectionState(STATE2);
    curIntersectionState = STATE2;
  }
  else if (timerSeconds >= stateThreeTime && timerSeconds < stateFourTime) {
    setIntersectionState(STATE3);
    curIntersectionState = STATE3;
  }
  else if (timerSeconds >= stateFourTime && timerSeconds < maxClockTime) {
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

void advancedGreenFlashing( uint8_t pin, uint8_t stablePin)
{
  unsigned long currentMillis = timerMillis; // Get the current time

  // Check if it's time to blink the LED
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; // Save the last time we blinked the LED

    // Toggle the blink LED state
    blinkState = !blinkState;
    digitalWrite(pin, blinkState);
    
    // Keep the hold LED on
    digitalWrite(stablePin, HIGH);
  } 
} 

ISR(TIMER1_COMPA_vect){
  timerSeconds++;
}

ISR(TIMER2_COMPA_vect){
  timerMillis++;
}