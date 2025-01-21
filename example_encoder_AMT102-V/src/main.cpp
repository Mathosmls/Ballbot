#include <Arduino.h>
#include <SimpleFOC.h>
#include <PciManager.h> //not useful for real interrupts
#include <PciListenerImp.h> //not useful for real interrupts


// !!!!!!!!!!!!
// I use software interrupt here
// !!!!!!!!!!!!


const int NUM_ENCODERS = 3;
Encoder encoders[NUM_ENCODERS] = {
    Encoder(2, 3, 2048),
    Encoder(5, 6, 2048),
    Encoder(20, 21, 2048)};

void doA0() { encoders[0].handleA(); }
void doB0() { encoders[0].handleB(); }
void doA1() { encoders[1].handleA(); }
void doB1() { encoders[1].handleB(); }
void doA2() { encoders[2].handleA(); }
void doB2() { encoders[2].handleB(); }

void (*doA[NUM_ENCODERS])() = {doA0, doA1, doA2};
void (*doB[NUM_ENCODERS])() = {doB0, doB1, doB2};

PciListenerImp *listenersA[NUM_ENCODERS]; //not useful for real interrupts
PciListenerImp *listenersB[NUM_ENCODERS];//not useful for real interrupts

void updateEncoders()
{
  for (int j = 0; j < NUM_ENCODERS; j++)
  {
    encoders[j].update(); // Met à jour les données de l'encodeur
  }
}
void printEncoderInfo()
{
  for (int j = 0; j < NUM_ENCODERS; j++)
  {
    Serial.print("enc ");
    Serial.print(j + 1);
    Serial.print(" : ");
    Serial.print(encoders[j].getAngle());
    Serial.print("\t");
    Serial.println(encoders[j].getVelocity());
  }
}

void setup()
{
  // put your setup code here, to run once:
  Serial.println("tst_example_Encoder_AMT102-V");
  Serial.begin(115200);
  Serial.println("Setup");

  for (int i = 0; i < NUM_ENCODERS; i++)
  {
    encoders[i].quadrature = Quadrature::ON;
    encoders[i].pullup = Pullup::USE_EXTERN;
    encoders[i].init();
    encoders[i].enableInterrupts(doA[i], doB[i]);
    // Associer les interruptions via PciListeners //not useful for real interrupts
    listenersA[i] = new PciListenerImp(encoders[i].pinA, doA[i]);
    listenersB[i] = new PciListenerImp(encoders[i].pinB, doB[i]);
    PciManager.registerListener(listenersA[i]);
    PciManager.registerListener(listenersB[i]);
  }
  Serial.println("Encoders ready");
}

void loop()
{
  static int i = 0;
  updateEncoders();
  if (i == 50)
  {
    printEncoderInfo();
    i = 0;
  }
  else
  {
    i++;
  }
}
