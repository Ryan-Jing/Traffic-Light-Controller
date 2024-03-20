#define RED_LED_PIN 9
#define GREEN_LED_PIN 10
#define YELLOW_LED_PIN 11
#define BUTTON_ONE_PIN 2
#define BUTTON_TWO_PIN 3

#define COMPARE_MATCH_REG(f) (16000000) / (f*1024) - 1
#define DEFAULT_FREQUENCY_HZ 1

volatile byte redLedState = LOW;
volatile byte greenLedState = LOW;
volatile byte yellowLedState = LOW;

uint32_t timeSeconds = 0;
uint8_t buttonOnePressed = 0;
uint8_t buttonTwoPressed = 0;

uint32_t redLightTimer = 5;

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
  traffic_light_states_t curState;
  traffic_light_inputs_t pedsInputs;
} traffic_light_t;

traffic_light_t myTrafficLight;
traffic_light_t myTrafficLight2;

void setup() {
  //set pins as outputs
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(BUTTON_ONE_PIN, INPUT);
  pinMode(BUTTON_TWO_PIN, INPUT);


  setTimer1Freq(DEFAULT_FREQUENCY_HZ);

  myTrafficLight.curState = RED;
  attachInterrupt(digitalPinToInterrupt(BUTTON_ONE_PIN), buttonOneInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(BUTTON_TWO_PIN), buttonTwoInterrupt, RISING);
}

void loop() {
  trafficLightController();
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

void buttonOneInterrupt() {
  // redLedState = !redLedState;
  // digitalWrite(RED_LED_PIN, redLedState);
  buttonOnePressed = 1;
}

void buttonTwoInterrupt() {
  buttonTwoPressed = 1;
}

void trafficLightController(){
  if(buttonOnePressed)
  {
    redLightTimer = 10;
    buttonOnePressed = 0;
  }

  switch (myTrafficLight.curState) {

    case GREEN:
      setTrafficeLight(YELLOW_LED_PIN, redLightTimer);
      myTrafficLight.curState = YELLOW;
      redLightTimer = 5;
      break;

    case YELLOW:
      setTrafficeLight(GREEN_LED_PIN, 2);
      myTrafficLight.curState = GREEN;
      break;

    case RED:
      setTrafficeLight(GREEN_LED_PIN, 5);
      myTrafficLight.curState = GREEN;
      if(buttonTwoPressed)
      {
        advancedGreenLight();
        break;
      }
      resetTimeSeconds();
      break;

    case ADVANCED_GREEN:
      resetTimeSeconds();
      myTrafficLight.curState = GREEN;
      break;
  }
}

void advancedGreenLight()
{
  myTrafficLight.curState = ADVANCED_GREEN;
  for(int i = 0; i < 5; i++)
  {
    digitalWrite(GREEN_LED_PIN, HIGH);
    delaySeconds(1);
    digitalWrite(GREEN_LED_PIN, LOW);
    delaySeconds(1);
  }
}
void setTrafficeLight(uint32_t colourLight, uint32_t timerSeconds)
{
  while(getTimeSeconds() < timerSeconds)
  {
    digitalWrite(colourLight, HIGH);
  }
  digitalWrite(colourLight, LOW);
}

void resetTimeSeconds()
{
  timeSeconds = 0;
}

int getTimeSeconds()
{
  return timeSeconds;
}

void delaySeconds( uint32_t delaySeconds )
{
  uint32_t startDelayTime = getTimeSeconds();
  while(getTimeSeconds() - startDelayTime < delaySeconds)
  {
    // do nothing
  }
}

ISR(TIMER1_COMPA_vect){//timer1 interrupt 1Hz toggles pin 13 (LED)
  //generates pulse wave of frequency 1Hz/2 = 0.5kHz (takes two cycles for full wave- toggle high then toggle low)
  timeSeconds++;
}
