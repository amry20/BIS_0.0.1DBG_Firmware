#define BIS_V1 0x00
#define BIS_V2 0x01
#define BIS_VERSION BIS_V1

#if BIS_VERSION == BIS_V1
    #define HEARTBEAT_LED PC_7
    #define STATUS_LED PB_6
    #define ALERT_SOUND PC_8
    #define AD9833_HIGH_FSYNC PB_13
    #define AD9833_LOW_FSYNC PB_12
    #define MUX_A0 PC_4
    #define MUX_A1 PC_5
    #define MUX_A2 PB_0
    #define MUX_EN PB_1
#else
#endif