#define RED_LED_PIN 9
#define GREEN_LED_PIN 10
#define YELLOW_LED_PIN 11
#define BUTTON_PIN 3

#define COMPARE_MATCH_REG(f) (16000000) / (f*1024) - 1
#define DEFAULT_FREQUENCY_HZ 1

volatile byte redLedState = LOW;
volatile byte greenLedState = LOW;
volatile byte yellowLedState = LOW;

uint32_t timeSeconds = 0;

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
  pinMode(BUTTON_PIN, INPUT);

  setTimer1Freq(DEFAULT_FREQUENCY_HZ);

  myTrafficLight.curState = RED;
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), blinkLed, RISING);
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

void blinkLed() {
  redLedState = !redLedState;
  digitalWrite(RED_LED_PIN, redLedState);
}

void trafficLightController(){
  switch (myTrafficLight.curState) {
    case RED:
      setTrafficeLight(RED_LED_PIN, 5);
      myTrafficLight.curState = YELLOW;
      break;
    case YELLOW:
      setTrafficeLight(YELLOW_LED_PIN, 2);
      myTrafficLight.curState = GREEN;
      break;
    case GREEN:
      setTrafficeLight(GREEN_LED_PIN, 5);
      resetTimeSeconds();
      myTrafficLight.curState = RED;
      break;
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


ISR(TIMER1_COMPA_vect){//timer1 interrupt 1Hz toggles pin 13 (LED)
//generates pulse wave of frequency 1Hz/2 = 0.5kHz (takes two cycles for full wave- toggle high then toggle low)
  timeSeconds++;
}
