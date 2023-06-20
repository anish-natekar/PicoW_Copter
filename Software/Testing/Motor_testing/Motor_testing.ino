#define MOT_TOP_LEFT 18
#define MOT_TOP_RIGHT 13
#define MOT_BOTTOM_LEFT 28
#define MOT_BOTTOM_RIGHT 1
void setup() {
  // put your setup code here, to run once:
  analogWriteFreq(500); // PWM frequency 500 Hz
  analogWriteRange(1000); // value coresponding to 100% PWM duty cycle

  pinMode(MOT_TOP_LEFT, OUTPUT);
  pinMode(MOT_TOP_RIGHT, OUTPUT);
  pinMode(MOT_BOTTOM_LEFT, OUTPUT);
  pinMode(MOT_BOTTOM_RIGHT, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  analogWrite(MOT_TOP_LEFT, 0);
  analogWrite(MOT_TOP_RIGHT, 0);
  analogWrite(MOT_BOTTOM_LEFT, 0);
  analogWrite(MOT_BOTTOM_RIGHT, 0);

  //Use the led on the Arduino for startup indication.
  digitalWrite(LED_BUILTIN,HIGH);
}
// take props off before uploading script
void loop() {
  // put your main code here, to run repeatedly:
  for(int i=0; i<=1000; i++) {
    analogWrite(MOT_TOP_RIGHT, i);
    analogWrite(MOT_BOTTOM_RIGHT, i);
    analogWrite(MOT_BOTTOM_LEFT, i);
    analogWrite(MOT_TOP_LEFT, i);
    delay(2);
  }
  for(int i=0; i<=1000; i++) {
    analogWrite(MOT_TOP_RIGHT, 1000-i);
    analogWrite(MOT_BOTTOM_RIGHT, 1000-i);
    analogWrite(MOT_BOTTOM_LEFT, 1000-i);
    analogWrite(MOT_TOP_LEFT, 1000-i);
    delay(2);
  }
}