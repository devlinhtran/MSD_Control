#include "string.h"
#include "math.h"

#define MSD_DT 0.001      //(s)
#define MSD_PPR 200.0     //This number is defined in software (Pulse/Row).
#define MSD_NUMBER  5     // Number of Motor want to control.
#define MSD_PULSE_DISTANCE 6.28318



class MSD_OBJ
{
  private:
    char start;
    char pause;
    typedef struct _MSD {
      double positionSet;
      double positionNext;
      double positionCurrent;
      double velocitySet;
      double velocityNext;
      double acerSet;
      long numberPulse;
      long indext;
      unsigned char aBegin;
      unsigned char aStop;
      unsigned char busy;
      char dirPin;
      char pulsePin;
    } MSD;
    typedef struct _MSD_STATE {
      unsigned char busy;
      unsigned start;
    } MSD_STATE;

    MSD_STATE msdModuleState;
    MSD msd[MSD_NUMBER];
  public:
    MSD_OBJ(char a) {
      this->start = 0;
      this->pause = 0;

    }
    void MSD_AttachPin(char motorIndex, char dirPin, char pulsePin);
    void MSD_Init(void);
    void MSD_StartAll(void);
    void MSD_StopAll(void);
    char MSD_StartAt(char index);
    char MSD_StopAt(char index);
    char MSD_Busy(void);
    void static MSD_Nop(unsigned long numberNop);
    double MSD_Abs(double x);
    void MSD_Scan();
    void MSD_GenerateSinal(void);
    void MSD_SetValue(double position, double velocity, double acer, char msdChanel);
    double MSD_GetPositionCurrent(char msdChanel);
    double MSD_GetVelocityCurrent(char msdChanel);
    void MSD_Run(void);
};

void MSD_OBJ::MSD_AttachPin(char motorIndex, char dirPin, char pulsePin)
{
  if (motorIndex < MSD_NUMBER) {
    msd[motorIndex].dirPin = dirPin;
    msd[motorIndex].pulsePin = pulsePin;
  }
  msdModuleState.start = 0x00;
}

void MSD_OBJ::MSD_Init(void) {
  int i;
  for (i = 0; i < MSD_NUMBER; i++) { //Config GPIO
    //Config as Output:
    pinMode(msd[i].dirPin, OUTPUT);
    pinMode(msd[i].pulsePin, OUTPUT);
  }
  //Config a TimerTick with frequency == MSD_DT
  cli();//aStop interrupts
  //set timer0 interrupt at 1kHz
  TCCR0A = 0;// set entire TCCR0A register to 0
  TCCR0B = 0;// same for TCCR0B
  TCNT0  = 0;//initialize counter value to 0
  // set compare match register for 1khz increments
  OCR0A = 248;// = (16*10^6) / (2000*64) - 1 (must be <256)
  // turn on CTC mode
  TCCR0A |= (1 << WGM01);
  // Set CS01 and CS00 bits for 64 prescaler
  TCCR0B |= (1 << CS01) | (1 << CS00);
  // enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);

  sei();//allow interrupts

}

void MSD_OBJ::MSD_StartAll(void) {
  char i;
  msdModuleState.start = 0x01;
  msdModuleState.busy = MSD_NUMBER;
  for (i = 0; i < MSD_NUMBER; i++) {
    msd[i].aStop = 0;
    msd[i].busy = 1;
  }
}
void MSD_OBJ::MSD_StopAll(void) {
  char i;
  msdModuleState.start = 0x00;
  msdModuleState.busy = 0;
  for (i = 0; i < MSD_NUMBER; i++) {
    msd[i].aStop = 1;
    msd[i].busy = 0;
  }
}
char MSD_OBJ::MSD_StartAt(char index) {
  if (index < MSD_NUMBER) {
    msd[index].aStop = 0;
    msd[index].busy = 1;
    msdModuleState.start = 1;
    return 0;
  }
  return 1;
}
char MSD_OBJ::MSD_StopAt(char index) {
  if (index < MSD_NUMBER) {
    msd[index].aStop = 1;
    return 0;
  }
  return 1;
}

char MSD_OBJ::MSD_Busy(void) {
  return msdModuleState.busy;
}
void MSD_OBJ::MSD_SetValue(double position, double velocity, double acer, char msdChanel) {
  msd[msdChanel].positionSet = position;
  msd[msdChanel].velocitySet = velocity;
  msd[msdChanel].acerSet = acer;
  msdModuleState.busy  += 0x01;
  msd[msdChanel].velocityNext = 0.0;
  msd[msdChanel].aBegin = 1;
  msd[msdChanel].numberPulse = 0;
}
void MSD_OBJ::MSD_Nop(unsigned long numberNop) {
  while (numberNop--);
}
void MSD_OBJ::MSD_GenerateSinal(void) {
  char index, countMsdFree = 0;
  if (msdModuleState.start == 0) return;
  for (index = 0; index < MSD_NUMBER; index++)
  {
    if (msd[index].aStop == 0) {
      while (msd[index].numberPulse != 0) {
        if (msd[index].numberPulse > 0) {
          digitalWrite(msd[index].dirPin, HIGH);  //Dir pin
          digitalWrite(msd[index].pulsePin, HIGH); //Pulse pin
          digitalWrite(msd[index].pulsePin, LOW); //Pulse pin
          msd[index].positionCurrent += 6.28318 / MSD_PPR; //628.3
          msd[index].numberPulse--;
        }
        else if (msd[index].numberPulse < 0) {
          digitalWrite(msd[index].dirPin, LOW);     //Dir pin
          digitalWrite(msd[index].pulsePin, HIGH);  //Pulse pin
          digitalWrite(msd[index].pulsePin, LOW);   //Pulse pin
          msd[index].positionCurrent -= 6.28318 / MSD_PPR;
          msd[index].numberPulse++;
        }
      }
      if (msd[index].positionSet == msd[index].positionNext || msd[index].aStop == 1) {
        msd[index].busy = 0;
        msd[index].aStop = 1;    //finish thi motor.
        countMsdFree++;
      }
    }

  }
  msdModuleState.busy = MSD_NUMBER - countMsdFree;
  if (msdModuleState.busy == 0) msdModuleState.start = 0;

}

double MSD_OBJ::MSD_Abs(double x) {
  if (x < 0) return -1.0 * x;
  else return x;
}
//This funtion have to call every MSD_DT time;
void MSD_OBJ::MSD_Scan() {
  int i, numberFinish;
  double s12;

  if ( msdModuleState.start) {
    for (i = 0; i < MSD_NUMBER; i++) {
      if (msd[i].aStop == 0) {
        //Noi suy tim diem giam toc:
        if ( msd[i].aBegin ) {
          msd[i].aBegin = 0x00;
          //Tim diem thoi gian giam toc:
          s12 = MSD_Abs(msd[i].positionSet - msd[i].positionCurrent) - msd[i].velocitySet * msd[i].velocitySet / msd[i].acerSet;
          if (s12 > 0) {
            msd[i].indext = (long)((msd[i].velocitySet / msd[i].acerSet + s12 / msd[i].velocitySet) / MSD_DT);
          }
          else {
            msd[i].indext = (long)(sqrt(MSD_Abs(msd[i].positionSet - msd[i].positionCurrent) / msd[i].acerSet) / MSD_DT);
          }
          //msd[i].indext= msd[i].indext*1.1;
        }

        if (msd[i].indext) { //tang toc:
          msd[i].velocityNext += msd[i].acerSet * MSD_DT;
          if (msd[i].velocityNext > msd[i].velocitySet) msd[i].velocityNext = msd[i].velocitySet;
          msd[i].indext--;
        }
        else { //giam toc:
          msd[i].velocityNext -= msd[i].acerSet * MSD_DT;
          if (msd[i].velocityNext < 3.14) msd[i].velocityNext = 3.14;
        }

        /////////////////////////////////////////
        //noi suy vi tri:
        if (msd[i].positionSet >= msd[i].positionCurrent) {
          msd[i].positionNext += msd[i].velocityNext * MSD_DT;
          if (msd[i].positionNext >= msd[i].positionSet) {
            msd[i].positionNext = msd[i].positionSet;
          }
        }
        else {
          msd[i].positionNext -= msd[i].velocityNext * MSD_DT;
          if (msd[i].positionNext <= msd[i].positionSet) {
            msd[i].positionNext = msd[i].positionSet;
          }
        }
        //ket qua duoc tra ve
        msd[i].numberPulse = ((msd[i].positionNext -  msd[i].positionCurrent) * MSD_PPR) / 6.28318;
      }
    }
    MSD_GenerateSinal();
  }
}

double MSD_OBJ::MSD_GetPositionCurrent(char msdChanel) {
  if (msdChanel < MSD_NUMBER) {
    return msd[msdChanel].positionCurrent;
  }
}
double MSD_OBJ::MSD_GetVelocityCurrent(char msdChanel) {
  if (msdChanel < MSD_NUMBER) {
    return msd[msdChanel].velocityNext;
  }
}
void MSD_OBJ::MSD_Run(void)
{
  static unsigned char mini_step = 0;
  switch (mini_step) {
    case 0:
      if (msd[0].aStop == 1)
      {
        MSD_SetValue(6.28318 * 2000, 150, 10, 0);
        MSD_StartAll();
        mini_step = 1;
      }
      break;
    case 1:
      if (msd[0].aStop == 1)
      {
        MSD_SetValue(MSD_PULSE_DISTANCE * -2000, 50, 5, 0);
        MSD_StartAll();
        mini_step = 0;
      }
      break;

  }
}



MSD_OBJ msdObj = MSD_OBJ(1);
void setup() {
  // put your setup code here, to run once:
  msdObj.MSD_AttachPin(0, 2, 3);
  msdObj.MSD_StopAll();
  msdObj.MSD_Init();
}

void loop() {
  // put your main code here, to run repeatedly:
  msdObj.MSD_Run();

}

//timer0 interrupt 1kHz,
ISR(TIMER0_COMPA_vect) { //please add this function
  msdObj.MSD_Scan();
}
