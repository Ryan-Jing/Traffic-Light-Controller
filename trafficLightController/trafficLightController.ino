#define RED_LED_PIN 9
#define GREEN_LED_PIN 10
#define BUTTON_PIN 3
volatile byte redLedState = LOW;
volatile byte greenLedState = LOW;


void setup() {
  //set pins as outputs
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);

  cli();//stop interrupts

  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei();

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), blinkLed, RISING);

}

void loop() {

}

void blinkLed() {
  redLedState = !redLedState;
  digitalWrite(RED_LED_PIN, redLedState);
}


ISR(TIMER1_COMPA_vect){//timer1 interrupt 1Hz toggles pin 13 (LED)
//generates pulse wave of frequency 1Hz/2 = 0.5kHz (takes two cycles for full wave- toggle high then toggle low)
  if (greenLedState){
    digitalWrite(GREEN_LED_PIN,HIGH);
    greenLedState = 0;
  }
  else{
    digitalWrite(GREEN_LED_PIN,LOW);
    greenLedState = 1;
  }
}
