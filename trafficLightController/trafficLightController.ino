/* Pin definitions */
#define RED_LED_PIN_HORIZONTAL 8
#define GREEN_LED_PIN_HORIZONTAL 9
#define YELLOW_LED_PIN_HORIZONTAL 10
#define RED_LED_PIN_VERTICAL 11
#define GREEN_LED_PIN_VERTICAL 12
#define YELLOW_LED_PIN_VERTICAL 13

#define BUTTON_PIN_HORIZONTAL 2
#define BUTTON_PIN_VERTICAL 3
#define ADVDANCED_GREEN_HORIZONTAL 4
#define ADVDANCED_GREEN_VERTICALs 5

/* Timer macros */
#define COMPARE_MATCH_REG(f) (16000000) / (f*1024) - 1
#define DEFAULT_FREQUENCY_HZ 1

#define STATE_ONE_TIME_DEFAULT 11
#define STATE_TWO_TIME_DEFAULT 7
#define STATE_THREE_TIME_DEFAULT 5
#define STATE_FOUR_TIME_DEFAULT 1
#define MAX_CLOCK_TIME_DEFAULT 12

uint32_t timerSeconds = 0; // Counter to keep track of the number of seconds that have elapsed

uint8_t stateOneTime = STATE_ONE_TIME_DEFAULT;
uint8_t stateTwoTime = STATE_TWO_TIME_DEFAULT;
uint8_t stateThreeTime = STATE_THREE_TIME_DEFAULT;
uint8_t stateFourTime = STATE_FOUR_TIME_DEFAULT;
uint8_t maxClockTime = MAX_CLOCK_TIME_DEFAULT;

bool pedestrianButtonHorizontalPressed = false;
bool pedestrianButtonVerticalPressed = false;


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

  pinMode(BUTTON_PIN_HORIZONTAL, INPUT);
  pinMode(BUTTON_PIN_VERTICAL, INPUT);

  setTimer1Freq(DEFAULT_FREQUENCY_HZ);

  Serial.begin(9600);

  // Set the initial state

  setIntersectionState(STATE1);

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN_HORIZONTAL), pedestrianButtonHorizontal, RISING);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN_VERTICAL), pedestrianButtonVertical, RISING);
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

void loop() {
  trafficLightController();
}

/* USER CODE BEGINS HERE */

void pedestrianButtonHorizontal() {
  /* DO SOME STUFF HERE TO CHANGE THE TIMERS */
  pedestrianButtonHorizontalPressed = true;
  Serial.println("----");
}

void pedestrianButtonVertical() {
  /* DO SOME STUFF HERE TO CHANGE THE TIMERS */
  pedestrianButtonVerticalPressed = true;
  Serial.println("++++");
}

void setIntersectionState(traffic_light_intersection_states_t intersectionState) {
  resetLights(); /* Reset all the lights before setting them again */

  switch(intersectionState) {
    case STATE1:
      if(pedestrianButtonHorizontalPressed == true) {
        digitalWrite(GREEN_LED_PIN_HORIZONTAL, HIGH);
        digitalWrite(RED_LED_PIN_VERTICAL, HIGH);
        delay(6000);
        //delaySeconds(6);
        maxClockTime = maxClockTime + 6;
        pedestrianButtonHorizontalPressed = false;
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
      if(pedestrianButtonVerticalPressed == true) {
        digitalWrite(RED_LED_PIN_HORIZONTAL, HIGH);
        digitalWrite(GREEN_LED_PIN_VERTICAL, HIGH);
        delay(6000);
        //delaySeconds(6);
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

void delaySeconds(uint32_t delayValueSeconds) {
  if(delayValueSeconds == 0)
  {
    return;
  }

  uint32_t startTime = timerSeconds;

  while (delayValueSeconds > 0)
  {
    yield();
    while (delayValueSeconds > 0 && (timerSeconds - startTime) >= 1)
    {
      delayValueSeconds--;
      startTime += 1;
    }
  }
}
ISR(TIMER1_COMPA_vect){//timer1 interrupt 1Hz toggles pin 13 (LED)
//generates pulse wave of frequency 1Hz/2 = 0.5kHz (takes two cycles for full wave- toggle high then toggle low)
  timerSeconds++;
}