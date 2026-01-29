#include "esc.h"
#include "main.h"
#include "stdio.h"

extern TIM_HandleTypeDef htim3;

uint8_t setSpeed(uint16_t speed);

uint8_t setSequence(uint16_t Sequencenum);

uint8_t senseEMF(Phase F);

static uint8_t currentSeq = 1;
static Phase currentFloating = U;

/* Controlling High and Low mosfets */
//High mosfet Off
static void highOff(Phase p)
{
    switch(p) {
        case U:
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);//PB1 - TIM3_CH4
        break;
        case V:
         __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);//PA7 - TIM3_CH2
        break;
        case W:
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);//PA6 - TIM3_CH1
        break;
    }
}
//High mosfet On needs PWM (duty)
static void highOn(Phase p, uint16_t duty) 
{
    switch(p) {
        case U:
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, duty);
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
        break;
        case V:
         __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty);
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
        break;
        case W:
         __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty);
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
        break;
    }
}

//Low mosfet off GPIO
static void lowOff(Phase p) //GPIO RESET = 0
{
    switch(p) {
        case U:
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);//PF0
        break;
        case V:
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);//PF1
        break;
        case W:
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);//PA0
        break;
    }
}
//Low mosfet on GPIO
static void lowOn(Phase p) //GPIO RESET = 1
{
    switch(p) {
        case U:
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET);
        break;
        case V:
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
        break;
        case W:
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
        break;
    }
}

/* Defining 6 sequences*/
void ESC_Init(void) //initialize all phases to floating
{
    highOff(U); highOff(V); highOff(W);
    lowOff(U); lowOff(V); lowOff(W);
}
Phase Pick_sequence(uint8_t num,uint16_t duty){
    ESC_Init(); //Possibly put into each case if too slow
    Phase floating = U;
    switch (num) {
    case 1: //U high, V low, 
        highOn(U, duty);
        lowOn(V);
        floating = W;
        break;
    case 2: //U high, W low
        highOn(U, duty);
        lowOn(W);
        floating = V;
        break;
    case 3: //V high, W low
        highOn(V, duty);
        lowOn(W);
        floating = U;
        break;
    case 4: //V high, U low
        highOn(V, duty);
        lowOn(U);
        floating = W;
        break;
    case 5: //W high, U low
        highOn(W, duty);
        lowOn(U);
        floating = V;
        break;
    case 6: //W high,V low
        highOn(W, duty);
        lowOn(V);
        floating = U;
        break;
    }
    return floating; 
}

/*Back-EMF, read the pins of the comparator*/

uint8_t senseBEMF(Phase F)
{
    GPIO_PinState s = GPIO_PIN_RESET; //pinstate holds 0 or 1, initialize 0

    switch(F)
    {
        case U:
        s = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);  //returns 0 or 1
        break;
        case V:
        s = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);
        break;
        case W:
        s = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
    }
    if (s == GPIO_PIN_SET)
        return 1;
    else
        return 0;
}

uint8_t detectZeroCrossing(Phase phase) //detect zero crossings of Phase U,V, W
{
    static uint8_t lastState[3] = {2, 2, 2}; //last state for seperate phases
    uint8_t phaseIndex = (uint8_t)phase; //setting U=0, V=1, W=2
    uint8_t currentState = senseBEMF(phase); //current state is sensed BEMF (0 or 1)
    uint8_t crossingDetected = 0;
    /*Detect zero crossing by checking previous and last state.
    Zero crossing = Low -> High or High -> Low transition of sensed BEMF */

    if (lastState[phaseIndex] == 2) //figure out if first crossing is 0 or 1
    {
        lastState[phaseIndex] = currentState;
        return 0;
    }

    if (lastState[phaseIndex] != currentState)
    {
        crossingDetected = 1; //zero crossing occured
        //printf("ZC on phase %d, state = %d\n", phaseIndex, currentState);
    }
    lastState[phaseIndex] = currentState;
    return crossingDetected; 
}









//CODE FOR SMOOTH 6 SEQUENCES TRANSITION (ONCE BEMF IS KNOWN)

/*Check if zerocrossing occured for phase
-> If yes then update tick time */
void ESC_UpdateZCTime (Phase F) { //Updates ticks when zero crossing occurs
    if (detectZeroCrossing(F)) {
        uint32_t now = ESC_GetTicks();
        ESC_findSequencePeriod(now);
    }
}


//GLOBAL VARIABLES

/*Keep track 360 electrical degree period. 6 sequences, each sequence 60 deg
I dont know how long it takes between each sequence ->*/
static uint32_t lastStepTick = 0; //last time ZC occcured
static uint32_t stepPeriodTicks = 0; //How many ticks for one 60 deg sequence
static uint32_t nextTimeTicks = 0; //At which tick for next sequence 
static uint8_t haveStepPeriod = 0; //0 unknown period, 1 found period

uint32_t ESC_GetTicks(void)
{
    return HAL_GetTick(); //starts at 0 on reset then increments every 1ms
}




void ESC_findSequencePeriod(uint32_t currentTicks) //passing through current time from UpdateZCtime func
{
    if (lastStepTick == 0) {  //No zero crossing yet
        lastStepTick = currentTicks; //Set first laststep to current tick(first zero crossing)
        haveStepPeriod = 0;
        return;
    }
    uint32_t delta = currentTicks - lastStepTick; //time between two zero-crossings
    //Update for next time -> this ZC becomes "Last ZC"
    lastStepTick = currentTicks;
    uint32_t newPeriod = 2 * delta;  //Full ticks for one 60 degree sequence is twice the zero crossing time(30 deg)
    if (!haveStepPeriod) { //first valid period 
        stepPeriodTicks = newPeriod; 
        haveStepPeriod =1; 

        //printf("First period found: %lu ticks\r\n", (unsigned long)newPeriod);
    } else {
        //75% old period and 25% new to nudge period to new period without jitter
        stepPeriodTicks = (stepPeriodTicks * 3 + newPeriod) / 4;
    }
    
}


//TEST FUNCTIONS

void ESC_PreCharge(void){
    highOff(U); highOff(V); highOff(W);
    lowOn(U); lowOn(V); lowOn(W);
    HAL_Delay(1); //1ms delay
    lowOff(U); lowOff(V); lowOff(W);
}



void Delay_1us(void){
    uint32_t n = 8;
    while (n--)
    {
        __NOP();
    }
}

void Delay_us(uint32_t us){
    while (us--)
    {
        Delay_1us();
    }
    
}

void ESC_Test_Seq1(void){
     uint16_t duty = 35;
     Pick_sequence(5, duty); //run first seuqence at duty
}

//htim3 period  = 399 (100% duty)
//Open loop (Initialize to find back EMF)
 void ESC_Test_OneSeq24V(void) { //Goal: See motor spin to one position
    uint32_t delay = 575;
    uint16_t duty =15; 
    //35 duty, 575 delay for 24V and 1250 delay with prop
    //35 duty, 760 delay  for 18V
    Pick_sequence(1, duty); //run first seuqence at duty
    Delay_us(delay);
    Pick_sequence(2, duty);
    Delay_us(delay);
    Pick_sequence(3, duty);
    Delay_us(delay);
    Pick_sequence(4, duty);
    Delay_us(delay);
    Pick_sequence(5, duty);
    Delay_us(delay);
    Pick_sequence(6, duty);
    Delay_us(delay);
} 

 void ESC_Test_OneSeq18V(void) { //Goal: See motor spin to one position
    uint32_t delay = 760;
    uint16_t duty = 35; 
    //35 duty, 575 delay for 24V
    //35 duty, 760 delay  for 18V
    Pick_sequence(1, duty); //run first seuqence at duty
    Delay_us(delay);
    Pick_sequence(2, duty);
    Delay_us(delay);
    Pick_sequence(3, duty);
    Delay_us(delay);
    Pick_sequence(4, duty);
    Delay_us(delay);
    Pick_sequence(5, duty);
    Delay_us(delay);
    Pick_sequence(6, duty);
    Delay_us(delay);
} 


void ESC_StartupOpenLoop(uint16_t duty) {
    uint8_t seq = 1;
    uint32_t delay = 400; //us
    uint32_t delay_min = 50; //us
    uint8_t lastseq = seq;
    Phase floating = U;
    haveStepPeriod = 0;
    lastStepTick = 0;

    while (!haveStepPeriod) { //Keep checking until a period is found
        floating = Pick_sequence(seq,duty);  //Drive seq 1
        lastseq = seq;
        uint32_t time = 0;
        while (time < delay && !haveStepPeriod) { //Constantly check BACK EMF
            Delay_us(10);
            ESC_UpdateZCTime(floating); //Listen to BEMF on floating phase and find time it takes
            time += 10;
        }
        Delay_us(delay); //Wait before next seq
        
        seq++; //move to next sequence
        if (seq > 6) seq = 1; //reset after 6th sequence
        if (delay > delay_min) {
            delay -= 5;
        }

    }
    currentSeq = lastseq;
    currentFloating = floating;
    //Once while loop exits I have period between sequences to start Closed loop
}

//Period found from OpenLoop -> Start Closed loop
void ESC_RunClosedLoop(uint16_t duty) {
    //uint8_t currentSeq = 1; //Start first seuqence 
    Phase floating = currentFloating; //get floating to monitor
    Pick_sequence(currentSeq, duty); 
    uint32_t now = ESC_GetTicks();
    nextTimeTicks = now + stepPeriodTicks; //Target time for next seq -> since reset + 60deg period
    while (1){
        now = ESC_GetTicks();
        ESC_UpdateZCTime(floating); //keep refreshing and finding BEMF period incase perioid changes (24V drops voltage bec of battery)
        /* Once now reaches or passes nextTime (Target seq switch),
        -> switch to next sequence */
        if ((int32_t)(now -nextTimeTicks)>= 0){ 
            //Start switching sequences
            currentSeq++;
            if (currentSeq > 6) currentSeq = 1;
                floating = Pick_sequence(currentSeq, duty);
                currentFloating = floating;
                nextTimeTicks += stepPeriodTicks; //schedule next seq time
        }
    }
}
