#include <stdint.h>

typedef enum {
    U,
    V,
    W,
} Phase;



uint8_t setSpeed(uint16_t speed);

uint8_t setSequence(uint16_t Sequencenum);

uint8_t senseEMF(Phase F);




void ESC_Init(void);
Phase Pick_sequence(uint8_t,uint16_t);
void ESC_findSequencePeriod(uint32_t currentTicks);
uint32_t ESC_GetTicks(void);
void ESC_Test_Seq1(void);
void ESC_Test_OneSeq24V(void);
void ESC_Test_OneSeq18V(void);
void ESC_PreCharge(void);
void ESC_StartupOpenLoop(uint16_t duty);
void ESC_RunClosedLoop(uint16_t duty);
