                                                                      /*                                                               
                                       (banner font: aciiart.eu)
  _____________________________________________________________
 |       _    __ _            _                                \
 |      / \  / _| |_ ___  _ _| |__  _   _ _ __ ___  ___  _ _   |\
 |     / _ \| |_| '_/ _ \| '_/ '_ \| | | | '_/  _ \/ _ \| '_/  ||
 |    / ___ \  _| |_| __/| | | |_) | |_| | | | | | | __/| |    ||
 |   /_/   \_\|  \__\___||_| |____/\___,_|_| |_| |_|___||_|    ||     
 \_____________________________________________________________||
   '------------------------------------------------------------'

   Afterburner: GAL IC Programmer for Arduino by -= olin =-

   Based on ATFblast 3.1 by Bruce Abbott 
     http://www.bhabbott.net.nz/atfblast.html
   
   Based on GALBLAST by Manfred Winterhoff
     http://www.armory.com/%7Erstevew/Public/Pgmrs/GAL/_ClikMe1st.htm

   Supports:
   * National GAL16V8
   * Lattice GAL16V8A, GAL16V8B, GAL16V8D
   * Lattice GAL22V10B
   * Atmel ATF16V8B, ATF22V10B, ATF22V10CQZ, ATF750C

   Requires:
   * afterburner PC program to upload JED fuse map, erase, read etc.
   * simple programming circuit. See: http://molej.cz/index_aft.html

   Changelog:
   * 2019.02.02 - initial version 0.1
   * 2019.04.09 - v. 0.3, 
                - added set & check of gal type,
                - fixed ATF22V10 and GAL22V10 CFG reading bug (porting bug)
   * 2020-07-31 - v. 0.4
                - added support for ATF750C

                                                                       */


#include "Arduino.h"

#define VERSION "0.4"

//#define DEBUG_PES
#define DEBUG_VERIFY

//ARDUINO UNO pin mapping
//    GAL PIN NAME | ARDUINO UNO PIN NUMBER

//programing voltage control pin
#define PIN_VPP        11
#define PIN_SDOUT      12
#define PIN_STROBE     13
#define PIN_PV         9
#define PIN_SDIN       8


#define PIN_RA0         10
#define PIN_RA1         2
#define PIN_RA2         3
#define PIN_RA3         4
#define PIN_RA4         5
#define PIN_RA5         6
#define PIN_SCLK        7




#define COMMAND_NONE 0
#define COMMAND_UNKNOWN 1
#define COMMAND_IDENTIFY_PROGRAMMER '*'
#define COMMAND_HELP 'h'
#define COMMAND_UPLOAD 'u'
#define COMMAND_DEBUG 'd'
#define COMMAND_READ_PES 'p'
#define COMMAND_READ_FUSES 'r'
#define COMMAND_READ_FUSES_RANGE 'R'
#define COMMAND_WRITE_FUSES 'w'
#define COMMAND_WRITE_FUSES_RANGE 'W'
#define COMMAND_VERIFY_FUSES 'v'
#define COMMAND_VERIFY_FUSES_RANGE 'V'
#define COMMAND_ERASE_GAL 'c'
#define COMMAND_UTX '#'
#define COMMAND_ECHO 'e'
#define COMMAND_TEST_VOLTAGE 't'
#define COMMAND_SET_GAL_TYPE 'g'
#define COMMAND_ENABLE_CHECK_TYPE 'f'
#define COMMAND_DISABLE_CHECK_TYPE 'F'

#define READGAL 0
#define VERIFYGAL 1
#define READPES 2
#define SCLKTEST 3
#define WRITEGAL 4
#define ERASEGAL 5
#define ERASEALL 6
#define BURNSECURITY 7
#define WRITEPES 8
#define VPPTEST 9
#define INIT 100


// contents of pes[3]
// Atmel PES is text string eg. 1B8V61F1 or 3Z01V22F1
//                                 ^           ^
#define LATTICE 0xA1
#define NATIONAL 0x8F
#define SGSTHOMSON 0x20
#define ATMEL16 'V'
#define ATMEL22 '1'

typedef enum {
  UNKNOWN,
  GAL16V8,
  GAL20V8,
  GAL22V10,
  ATF16V8B,
  ATF22V10B,
  ATF22V10C,
  ATF750C,
  LAST_GAL_TYPE //dummy
} GALTYPE;

enum range_mem_type {
  none = 0,
  fuses = 1,
  ues = 2,
  cfg = 3,
  ext = 4
};

// config bit numbers

#define CFG_BASE_16 2048
#define CFG_BASE_20 2560
#define CFG_BASE_22 5808
#define CFG_BASE_750 14364
//#define CFG_BASE_750EXT 14395

#define CFG_SET_ROW 1
#define CFG_STROBE_ROW 2
#define CFG_STROBE_ROW2 3

// common CFG fuse address map for cfg16V8 and cfg20V8
// the only difference is the starting address: 2048 for cfg16V8 and 2560 for cfg20V8
// total size: 82
static const unsigned char cfgV8[]=
{
      80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,
      0,1,2,3,
      145,
      72,73,74,75,76,77,78,79,
      144,
      4,5,6,7,
      112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143,
};

// common CFG fuse address map for cfg16V8AB and cfg20V8AB
// the only difference is the starting address: 2048 for cfg16V8AB and 2560 for cfg20V8AB
// total size: 82
static const unsigned char cfgV8AB[]=
{
      0,1,2,3,
      145,
      72,73,74,75,
      80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143,
      76,77,78,79,
      144,
      4,5,6,7,
};

// common CFG fuse address map for cfg22V10
// starting address: 5808
static const unsigned char cfgV10[]=
{
      1,0,3,2,5,4,7,6,9,8,11,10,13,12,15,14,17,16,19,18,
      // TODO: is the value 84 for Powerdown fuse missing here?
};

static const unsigned char cfgV750[]=
{
    // TODO: order is not clear
#if 0
//not working 2020-08-12 13:09
   34, 33, 32, 31,     2,  1,  0, // 0  S6..S0
   //38, 37, 36, 35,     5,  4,  3, // 7
   3, 4, 5, 35, 36, 37, 38,            // S0..S6
   //42, 41, 40, 39,     8,  7,  6, // 14
   39, 40, 41, 42,    6, 7, 8,         // S3..S6, S0..S2
   //46, 45, 44, 43,    11, 10,  9, // 21
   11, 10, 9, 46, 45, 44, 43,         // S2..S0, S6..S3
   50, 49, 48, 47,    14, 13, 12, // 28
   54, 53, 52, 51,    17, 16, 15, // 35
   58, 57, 56, 55,    20, 19, 18, // 42
   62, 61, 60, 59,    23, 22, 21, // 49
   66, 65, 64, 63,    26, 25, 24, // 56
   70, 69, 68, 67,    29, 28, 27, // 63
   30,  // 75: Security?
#endif

#if 0
//not working 2020-08-12 13:09
   34, 33, 32, 31,     2,  1,  0, // 0 
   38, 37, 36, 35,     5,  4,  3, // 7
   42, 41, 40, 39,     8,  7,  6, // 14
   46, 45, 44, 43,    11, 10,  9, // 21
   50, 49, 48, 47,    14, 13, 12, // 28
   54, 53, 52, 51,    17, 16, 15, // 35
   58, 57, 56, 55,    20, 19, 18, // 42
   62, 61, 60, 59,    23, 22, 21, // 49
   66, 65, 64, 63,    26, 25, 24, // 56
   70, 69, 68, 67,    29, 28, 27, // 63
   30,  // 75: Security?
#endif

#if 1
// not working 2020-08-16 10:09
   34, 33, 32, 31,     2,  1,  0, // 0 
   38, 37, 36, 35,     5,  4,  3, // 7
   42, 41, 40, 39,     8,  7,  6, // 14
   46, 45, 44, 43,    11, 10,  9, // 21
   50, 49, 48, 47,    14, 13, 12, // 28
   54, 53, 52, 51,    17, 16, 15, // 35
   58, 57, 56, 55,    20, 19, 18, // 42
   62, 61, 60, 59,    23, 22, 21, // 49
   66, 65, 64, 63,    26, 25, 24, // 56
   70, 69, 68, 67,    29, 28, 27, // 63
     30,  // 75: Security?
    135, // 70: Powerdown
    136, // 71: PinKeeper
    137, // 72: unknown
    138, // 73: unknown
    139, // 74: unknown
#endif

#if 0
//not working 2020-08-13 13:09
   30,  // 75: Security?
   34, 33, 32, 31,     2,  1,  0, // 0 
   38, 37, 36, 35,     5,  4,  3, // 7
   42, 41, 40, 39,     8,  7,  6, // 14
   46, 45, 44, 43,    11, 10,  9, // 21
   50, 49, 48, 47,    14, 13, 12, // 28
   54, 53, 52, 51,    17, 16, 15, // 35
   58, 57, 56, 55,    20, 19, 18, // 42
   62, 61, 60, 59,    23, 22, 21, // 49
   66, 65, 64, 63,    26, 25, 24, // 56
   70, 69, 68, 67,    29, 28, 27, // 63
#endif

#if 0
//not working 2020-08-12 13:09
      2,  1,  0,   34, 33, 32, 31, // 0 offset
      5,  4,  3,   38, 37, 36, 35, // 7
      8,  7,  6,   42, 41, 40, 39, // 14
     11, 10,  9,   46, 45, 44, 43, // 21
     14, 13, 12,   50, 49, 48, 47, // 28
     17, 16, 15,   54, 53, 52, 51, // 35
     20, 19, 18,   58, 57, 56, 55, // 42
     23, 22, 21,   62, 61, 60, 59, // 49
     26, 25, 24,   66, 65, 64, 63, // 56
     29, 28, 27,   70, 69, 68, 67, // 63
     30,  // 75: Security?
#endif

#if 0
//not working 2020-08-16 08:29
      2,  1,  0,   34, 33, 32, 31, // 0 offset
      5,  4,  3,   38, 37, 36, 35, // 7
      8,  7,  6,   42, 41, 40, 39, // 14
     11, 10,  9,   46, 45, 44, 43, // 21
     14, 13, 12,   50, 49, 48, 47, // 28
     17, 16, 15,   54, 53, 52, 51, // 35
     20, 19, 18,   58, 57, 56, 55, // 42
     23, 22, 21,   62, 61, 60, 59, // 49
     26, 25, 24,   66, 65, 64, 63, // 56
     29, 28, 27,   70, 69, 68, 67, // 63
     30,  // 75: Security?
    135, // 70: Powerdown
    136, // 71: PinKeeper
    137, // 72: unknown
    138, // 73: unknown
    139, // 74: unknown
#endif

#if 0
//not working 2020-08-12 13:09
      2,  1,  0,
      5,  4,  3,
      8,  7,  6,
     11, 10,  9,
     14, 13, 12,
     17, 16, 15,
     20, 19, 18,
     23, 22, 21,
     26, 25, 24,
     29, 28, 27,

   34, 33, 32, 31, // 0 
   38, 37, 36, 35, // 7
   42, 41, 40, 39, // 14
   46, 45, 44, 43, // 21
   50, 49, 48, 47, // 28
   54, 53, 52, 51, // 35
   58, 57, 56, 55, // 42
   62, 61, 60, 59, // 49
   66, 65, 64, 63, // 56
   70, 69, 68, 67, // 63
   30,  // 75: Security?
#endif

#if 0
//not working 2020-08-12 13:09
      2,  1,  0,
      5,  4,  3,
      8,  7,  6,
     11, 10,  9,
     14, 13, 12,
     17, 16, 15,
     20, 19, 18,
     23, 22, 21,
     26, 25, 24,
     29, 28, 27,

   30,  // 75: Security?

   34, 33, 32, 31, // 0 
   38, 37, 36, 35, // 7
   42, 41, 40, 39, // 14
   46, 45, 44, 43, // 21
   50, 49, 48, 47, // 28
   54, 53, 52, 51, // 35
   58, 57, 56, 55, // 42
   62, 61, 60, 59, // 49
   66, 65, 64, 63, // 56
   70, 69, 68, 67, // 63
#endif

    // TODO: unclear how to handle those:
    /*
    30,  // 75: Security?
    135, // 70: Powerdown
    136, // 71: PinKeeper
    137, // 72: unknown
    138, // 73: unknown
    139, // 74: unknown
    */
};


//   UES     user electronic signature
//   PES     programmer electronic signature (ATF = text string, others = Vendor/Vpp/timing)
//   cfg     configuration bits for OLMCs

// GAL info 
static struct
{
    GALTYPE type;
    unsigned char id0,id1;          /* variant 1, variant 2 (eg. 16V8=0x00, 16V8A+=0x1A)*/
    const char *name;               /* pointer to chip name               */
    short fuses;                    /* total number of fuses              */
    char pins;                      /* number of pins on chip             */
    char rows;                      /* number of fuse rows                */
    unsigned char bits;             /* number of fuses per row            */
    char uesrow;                    /* UES row number                     */
    short uesfuse;                  /* first UES fuse number              */
    char uesbytes;                  /* number of UES bytes                */
    char eraserow;                  /* row adddeess for erase             */
    char eraseallrow;               /* row address for erase all          */
    char pesrow;                    /* row address for PES read/write     */
    char pesbytes;                  /* number of PES bytes                */
    char cfgrow;                    /* row address of config bits         */
    unsigned short cfgbase;         /* base address of the config bit numbers */
    const unsigned char *cfg;       /* pointer to config bit numbers      */
    unsigned char cfgbits;          /* number of config bits              */
    unsigned char cfgmethod;        /* strobe or set row for reading config */
}
galinfo[]=
{
//                                      + fuses         + bits       +uesbytes   +pesrow          +cfgbase
//                                      |     +pins     |  +uesrow   |  +eraserow|   +pesbytes    |        +cfg
//   +-- type   + id0 + id1  +- name    |     |   +rows |  |   +uesfuse |   +eraseallrow +cfgrow  |        |       + cfgbits        +cfgmethod
//   |          |     |      |          |     |   |     |  |   |     |  |   |    |   |   |        |        |       |                |
    {UNKNOWN,   0x00, 0x00, "unknown",     0,  0,  0,   0,  0,    0, 0,  0,  0,  0,  8,  0,       0,        NULL,      0         , 0},
    {GAL16V8,   0x00, 0x1A, "GAL16V8",  2194, 20, 32,  64, 32, 2056, 8, 63, 54, 58,  8, 60, CFG_BASE_16, cfgV8AB, sizeof(cfgV8AB), CFG_STROBE_ROW}, 
    {GAL20V8,   0x20, 0x3A, "GAL20V8",  2706, 24, 40,  64, 40, 2568, 8, 63, 59, 58,  8, 60, CFG_BASE_20, cfgV8AB, sizeof(cfgV8AB), CFG_STROBE_ROW}, 
    {GAL22V10,  0x48, 0x49, "GAL22V10", 5892, 24, 44, 132, 44, 5828, 8, 61, 60, 58, 10, 16, CFG_BASE_22, cfgV10,  sizeof(cfgV10) , CFG_SET_ROW   },
    {ATF16V8B,  0x00, 0x00, "ATF16V8B", 2194, 20, 32,  64, 32, 2056, 8, 63, 54, 58,  8, 60, CFG_BASE_16, cfgV8AB, sizeof(cfgV8AB), CFG_STROBE_ROW},
    {ATF22V10B, 0x00, 0x00, "ATF22V10B",5892, 24, 44, 132, 44, 5828, 8, 61, 60, 58, 10, 16, CFG_BASE_22, cfgV10,  sizeof(cfgV10) , CFG_SET_ROW   },
    {ATF22V10C, 0x00, 0x00, "ATF22V10C",5892, 24, 44, 132, 44, 5828, 8, 61, 60, 58, 10, 16, CFG_BASE_22, cfgV10,  sizeof(cfgV10) , CFG_SET_ROW   },
    {ATF750C,   0x00, 0x00, "ATF750C", 14499, 24, 84, 171, 84,14435, 8, 61, 60,127, 10, 16, CFG_BASE_750,cfgV750, sizeof(cfgV750), CFG_STROBE_ROW2}, // TODO: not all numbers are clear
};

// MAXFUSES calculated as the biggest required space to hold the fuse bitmap + UES bitmap + CFG bitmap
// MAXFUSES = ((132 * 44 bits) / 8)  + uesbytes + (20 / 8)
//               726 + 8 + 3
#define MAXFUSES 737

// for ATF750C
// MAXFUSES = (((171 * 84 bits)  + uesbits + (10*3 + 1 + 10*4 + 5)) + 7) / 8
//               (14504 + 7) / 8 = 1813
//#define MAXFUSES 1813
// TODO: this size will not fit in the memory of the Arduino RAM (variable fusemap would get too large)

#define LINE_LEN (80)

GALTYPE gal; //the gal device index pointing to galinfo
//static short security = 0;
static short erasetime = 100, progtime = 100, vpp = 0;

char echoEnabled;
unsigned char pes[12];
char line[LINE_LEN];
char programName[32];
short lineIndex;
char endOfLine;
char mapUploaded;
char isUploading;
char uploadError;
unsigned char fusemap[MAXFUSES];
char typeCheck; //check GAL type before starting an operation
uint8_t rangeMemType; // TODO: move vars to beginning
uint8_t rangeStartRow;
uint8_t rangeRowCount;


static void setFuseBit(unsigned short bitPos);
static unsigned short checkSum(unsigned short n);
static char checkGalTypeViaPes(void);
static void turnOff(void);
static void printFormatedNumberHex2(unsigned char num) ;

// the number of bit of a row address
static uint8_t getRowAddrWidth() {
  if(galinfo[gal].rows >= 64)
    return 7;

  return 6;
}

static uint8_t bitReverse(uint8_t row)
{
  unsigned char n = getRowAddrWidth();
  uint8_t out = 0;
  for(uint8_t i = 0; i < n; i++) {
  	out = (out << 1) | (row & 0x01);
  	row >>= 1;
  }

  return out;
}


#define TEST750
#ifdef TEST750
void testRead() {
  switch(gal)
  {
    case ATF22V10C:
    case ATF750C:
      break;

    default:
      Serial.println(F("unsupported chip"));
      return;
      break;
  }

  const unsigned char* cfgArray = (gal == ATF750C) ? cfgV750 : cfgV10;
  char useDelay = 1;
  char doDiscardBits = galinfo[gal].bits - 8 * galinfo[gal].uesbytes;
  unsigned short cfgAddr = galinfo[gal].cfgbase;
  unsigned short row, bit;
  unsigned short addr;

  //const uint8_t maxrow = galinfo[gal].rows - 1;
  const uint8_t maxrow = (1 << getRowAddrWidth()) - 1;
  turnOn(READGAL);

  // read fuse rows
  for(row = 0; row <= maxrow; row++) {
    if(row < 10) Serial.write('0');
    if(row < 100) Serial.write('0');
    Serial.print(row, DEC);
    Serial.print(F(": "));
    strobeRow(row);
    for(bit = 0; bit < galinfo[gal].bits; bit++) {
      uint8_t v = receiveBit();
      Serial.write(v ? '1' : '0');
    }
    Serial.println();
    if (useDelay) {
      delayPrecise(useDelay);
    }
  }

  // read UES
  Serial.print(F("\nUES: "));
  strobeRow(galinfo[gal].uesrow);
  if (doDiscardBits) {
    discardBits(doDiscardBits);
  }
  for(bit = 0; bit < galinfo[gal].uesbytes * 8; bit++) {
    uint8_t v = receiveBit();
    Serial.write(v ? '1' : '0');
  }
  if (useDelay) {
    delayPrecise(useDelay);
  }
  Serial.println();

  // read CFG new 2, emulation of GALEP-4 algorithm
  for(uint8_t i = 32; i <= 41; i++) {
    Serial.write('Y');
    if(i < 10) Serial.write('0');
    Serial.print(i);
    Serial.print(F(": "));
    if (galinfo[gal].cfgmethod == CFG_STROBE_ROW) {
      strobeRow(i);
    }
    else if (galinfo[gal].cfgmethod == CFG_STROBE_ROW2) {
      strobeConfigRow(i);
    }
    else if (galinfo[gal].cfgmethod == CFG_SET_ROW) {
      setRow(i);
      strobe(3);
    }
    for(bit = 0; bit < 10; bit++) {
      uint8_t v = receiveBit();
      Serial.write(v ? '1' : '0');
    }
    //if (useDelay) {
    //  delayPrecise(useDelay);
    //}
    Serial.println();
  }
  Serial.println();

  // read CFG new
  for(uint8_t i = 0; i < 128; i++) {
    Serial.write('X');
    if(i < 10) Serial.write('0');
    Serial.print(i);
    Serial.print(F(": "));
    if (galinfo[gal].cfgmethod & CFG_STROBE_ROW) {
      strobeRow(i);
    }
    if (galinfo[gal].cfgmethod & CFG_SET_ROW) {
      setRow(i);
      strobe(3);
    }
    for(bit = 0; bit < galinfo[gal].cfgbits; bit++) {
      if(bit == 10 || bit == 11 || bit == 17) Serial.write(' ');
      uint8_t v = receiveBit();
      Serial.write(v ? '1' : '0');
    }
    if (useDelay) {
      delayPrecise(useDelay);
    }
    Serial.println();
  }
  Serial.println();

  // read CFG old
  for(uint8_t i = 0; i < 64; i++) {
    Serial.write('C');
    if(i < 10) Serial.write('0');
    Serial.print(i);
    Serial.print(F(": "));
    if (galinfo[gal].cfgmethod == CFG_STROBE_ROW) {
      strobeRow(i);
    }
    else {
      setRow(i);
      strobe(3);
    }
    for(bit = 0; bit < galinfo[gal].cfgbits; bit++) {
      uint8_t v = receiveBit();
      Serial.write(v ? '1' : '0');
    }
    if (useDelay) {
      delayPrecise(useDelay);
    }
    Serial.println();
  }

  turnOff();	
}
#endif


// print some help on the serial console
void printHelp(char full) {
  Serial.println(F("AFTerburner v." VERSION));
  if (!full) {
    Serial.println(F("type 'h' for help"));
    return;
  }
  Serial.println(F("commands:"));
  Serial.println(F("  h - print help"));
  Serial.println(F("  e - toggle echo")); 
  Serial.println(F("  p - read & print PES"));
  Serial.println(F("  r - read & print fuses"));  
  Serial.println(F("  u - upload fuses"));
  Serial.println(F("  w - write uploaded fuses"));
  Serial.println(F("  v - verify fuses"));
  Serial.println(F("  c - erase chip"));
  Serial.println(F("  t - test VPP"));
}

// setup the Arduino board
void setup() {
// initialize serial:
  Serial.begin(38400);
  isUploading = 0;
  endOfLine = 0;
  gal = ATF16V8B;
  echoEnabled = 0;
  mapUploaded = 0;
  typeCheck = 1; //do type check

  // Serial output from the GAL chip, input for Arduino
  pinMode(PIN_SDOUT, INPUT);
  // Serial input of the GAL chip, output from Arduino
  pinMode(PIN_SDIN, OUTPUT);

  pinMode(PIN_STROBE, OUTPUT);
  pinMode(PIN_PV, OUTPUT);
  pinMode(PIN_RA0, OUTPUT);
  pinMode(PIN_RA1, OUTPUT);
  pinMode(PIN_RA2, OUTPUT);
  pinMode(PIN_RA3, OUTPUT);
  pinMode(PIN_RA4, OUTPUT);
  pinMode(PIN_RA5, OUTPUT);
  pinMode(PIN_SCLK, OUTPUT);

  pinMode(PIN_VPP, OUTPUT);
  // Important - the output pins should be kept high.
  // TurnOff function does that (keeps pin high).
  turnOff();

  printHelp(0);
  Serial.println(">");
}


// read from serial line and discard the data
void readGarbage() {
  while (Serial.available() > 0) {
    Serial.read();
  }  
}

// Reads input from the serial terminal and returns the command
// which is the first character of the entered text. 

char handleTerminalCommands() {
  char c;
  
  while (Serial.available() > 0) {
    c = Serial.read();
    line[lineIndex] = c;
    if (c == '\n' || c == '\r') {
      endOfLine = 1;
    }
    //echo input to output
    else {
      if (!isUploading && echoEnabled) {
        Serial.print(c);
      }
    }
    if (lineIndex >= (LINE_LEN - 2)) {
      lineIndex = 0;
      readGarbage();
      Serial.println();
      Serial.println("Error: line too long.");
    } else {
      lineIndex++;
    }
  }
  if (endOfLine) {
    c = COMMAND_NONE;
    line[lineIndex] = 0;

    //single letter command entered
    if (lineIndex == 2) {
      c = line[0];  
    } else if (lineIndex  > 2) {
      c = line[0];  
      if (!isUploading || c != '#') {
        if ((   c != COMMAND_SET_GAL_TYPE)
            && (c != COMMAND_READ_FUSES_RANGE)
            && (c != COMMAND_WRITE_FUSES_RANGE)
            && (c != COMMAND_VERIFY_FUSES_RANGE)
        ) {
          c = COMMAND_UNKNOWN; 
        }
      }
    }
    if (!isUploading) {
      Serial.println();
      lineIndex = 0;
    }
    endOfLine = 0;
    return c;
  }
  return COMMAND_NONE; 
}

// Parses decimal integer number typed as 4 digit.
// Returns the number value.
unsigned short parse4dec(char i) {
  unsigned short v = (line[i++] - '0') * 1000;
  v += (line[i++] - '0') * 100;
  v += (line[i++] - '0') * 10;
  v += line[i] - '0';
  return v;
}

// Converts textual hex value 0-9, A-F to a number.
unsigned char toHex(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return 0;
}

// Parses hexdecimal integer number typed as 2 digit.
// Returns the number value.
unsigned short parse2hex(char i) {
  if (line[i] == '\r' || line[i] ==  0 || line[i] == ' ') {
    return -1; 
  }
  
  unsigned short v = toHex(line[i++]) << 4;
  return v + toHex(line[i]); 
}

// Parses hexdecimal integer number typed as 4 digit.
// Returns the number value.
unsigned short parse4hex(char i) {
  if (line[i] == '\r' || line[i] ==  0 || line[i] == ' ') {
    return -1; 
  }
  
  unsigned short v = ((unsigned short)toHex(line[i++])) << 12;
  v |= ((unsigned short)toHex(line[i++])) << 8;
  v |= toHex(line[i++]) << 4;
  return v + toHex(line[i]); 
}

// cmd format: "W <memType> <startRow> <rowCount>"
// numbers as 2 hex digits, e.g. "W 01 36 10"
void parseRangeOptions() {
  rangeMemType  = parse2hex(2);
  rangeStartRow = parse2hex(5);
  rangeRowCount = parse2hex(8);
}

void clearRangeOptions() {
  rangeMemType  = range_mem_type::none;
  rangeStartRow = 0;
  rangeRowCount = 0;
}

// Parses a line fed by the serial connection.
// This hnadles a primitive upload protocol that
// expects a programatic data feed - not suitable
// for human interaction.
// Data: each command on its own line
// line starts with '#' character followed by a command
// and a space. Then a command specific data follow.
// Commands:
// t <gal index>: gal type index to the GALTYPEE enum
// f <fuse index> <row>: row of fuse-map data starting on fuse bit index
// c <checksum> : checksum of the whole fuse map
// C <checksum> <number of fuses> : checksum of the specified range
// e : end ofthe upload transfer - returns to terminal

void parseUploadLine() {
  switch (line[1]) {
    case 'e': {
      if (uploadError) {
        Serial.print(F("ER upload failed"));
      } else {
        Serial.print(F("OK upload finished"));
      }
      isUploading = 0;
    } break;

    // gal type
    case 't': {
      short v = line[3] - '0';
      if (v > 0 && v < LAST_GAL_TYPE) {
        gal = (GALTYPE) v;
        Serial.print(F("OK gal set: "));
        Serial.println((short) gal, DEC);
      } else {
        Serial.println(F("ER unknown gal index"));
        uploadError = 1;
      }
    } break;

    //fusemap data
    case 'f': {
      uint16_t addr = parse4dec(3);
      int16_t byteCount = strlen(line);
      while(byteCount > 0 && (line[byteCount - 1] == '\r' || line[byteCount - 1] == '\n'))
        byteCount--;
      byteCount = (byteCount - 8) / 2;
      uint8_t pos = 8;
      for(uint8_t i = 0; i < byteCount; i++) {
        uint8_t v = parse2hex(pos);
        if (v) {
          for (uint8_t j = 0; j < 8; j++) {
            // if fuse bit is set -> then change the fusemap
            if (v & (1 << j)) {
              setFuseBit(addr + j);
            }
          }
        }
        pos += 2;
        addr += 8;
      }

      //any fuse being set is considered as uploaded fuse map
      mapUploaded = 1;

      Serial.print(F("OK "));
      Serial.println(addr, DEC);
    } break;

    //checksum
    case 'C': // has a second parameter: the number of fuses for the checksum calculation
    case 'c': {
      unsigned short val = parse4hex(3);
      unsigned short calc_cs = (line[1] == 'C') ? checkSum(parse4hex(8)) : checkSum(galinfo[gal].fuses);
      if (calc_cs == val) {
        Serial.println(F("OK checksum matches"));
      } else {
        uploadError = 1;
        Serial.print(F("ER checksum:"));
        Serial.print(calc_cs, HEX);
        Serial.print(F(" expected:"));
        Serial.println(val, HEX);
      }
    } break;
    
    default:
      uploadError = 1;
      Serial.println(F("ER unknown upload cmd"));
  }

  lineIndex = 0;
}
// *********************************************************


// set/reset individual pins of GAL 
static void setVCC(char on) {
    //no control for turning the voltage on of
    //it is assumed the voltage is always on
}

static void setVPP(char on) {
  //programming voltage is controlled by VPP_PIN,
  //but the programming voltage must be set manually by user turning a Pot
  digitalWrite(PIN_VPP, on ? 1 : 0);
  
  //Serial.print(F("VPP set to:"));
  //Serial.println( on ? "12V": "5V");
  delayPrecise(10);
}



static void setSTB(char on) {
   digitalWrite(PIN_STROBE, on ? 1:0);
}

static void setPV(char on) {
   digitalWrite(PIN_PV, on ? 1:0);
}

static void setSDIN(char on) {
  digitalWrite(PIN_SDIN, on ? 1:0);
}

static void setSCLK(char on){
   digitalWrite(PIN_SCLK, on ? 1:0);
}

// output row address (RA0-5)
static void setRow(char row)
{
  digitalWrite(PIN_RA0, (row & 0x1));
  digitalWrite(PIN_RA1, ((row & 0x2) ? 1:0));
  digitalWrite(PIN_RA2, ((row & 0x4) ? 1:0));
  digitalWrite(PIN_RA3, ((row & 0x8) ? 1:0));
  digitalWrite(PIN_RA4, ((row & 0x10) ? 1:0));
  digitalWrite(PIN_RA5, ((row & 0x20) ? 1:0));
}

// serial data out form the GAL chip -> received by Arduino
static char getSDOUT(void)
{
  return digitalRead(PIN_SDOUT) != 0;
}

// GAL finish sequence
static void turnOff(void)
{
    delayPrecise(100);
    setPV(0);    // P/V- low
    setRow(0x3F);// RA0-5 high  
    setSDIN(1);  // SDIN high
    setVPP(0);   // Vpp off (+12V)
    setPV(1);    // P/V- high
    delayPrecise(2);
    setVCC(0);   // turn off VCC (if controlled)

    
}

// GAL init sequence
static void turnOn(char mode) {

    if (
      mode == WRITEGAL ||
      mode == ERASEGAL ||
      mode == ERASEALL ||
      mode == BURNSECURITY ||
      mode == WRITEPES ||
      mode == VPPTEST ||
      mode == READPES ||
      mode == READGAL 
    ) {
        mode = 1;
    } else {
        mode = 0;
    }

//     setVPP(mode);
    setVPP(0);    // VPP off
    delayMicroseconds(100);
    setPV(0);     // P/V- low
    delayMicroseconds(100);
    setRow(0x3F); // RA0-5 high - erase sequence ?
    delayMicroseconds(100);
    //setRow(0);    // RA0-5 low
    //delayMicroseconds(100);
    setSDIN(1);   // SDIN high
    delayMicroseconds(100);
    setSCLK(1);   // SCLK high
    delayMicroseconds(100);
    setSTB(1);    // STB high
    delayMicroseconds(100);
    setVCC(1);    // turn on VCC (if controlled)
    delayPrecise(100);
    setSCLK(0);   // SCLK low
    delayMicroseconds(100);
    setVPP(mode);
    delayPrecise(20);
}


// clock and receive a bit in from GAL SDOUT
static char receiveBit(void)
{
    char b = getSDOUT();
    setSCLK(1);
    delayMicroseconds(12);
    setSCLK(0);
    delayMicroseconds(12);
    return b;
}

// read n number of bits
static void discardBits(short n)
{
    while (n-- > 0) {
      receiveBit();
    }
}

// clock a bit and send it out to GAL SDIN
static void sendBit(char bitValue)
{
    setSDIN(bitValue);
    delayMicroseconds(12);
    setSCLK(1);
    delayMicroseconds(12);
    setSCLK(0);
    delayMicroseconds(12);
}

// send n number of bits to GAL
static void sendBits(short n, char bitValue)
{
    while (n-- > 0) {
      sendBit(bitValue);
    }
}

// send row address bits to SDIN 
// ATF22V10C MSb first, ATF750C and other 22V10 LSb first
static void sendAddress(unsigned char row)
{
  unsigned char n = getRowAddrWidth();
 
  switch (gal) {
  case ATF750C:   // LSb first
	  	row = bitReverse(row);
      // fall through
  case ATF22V10C: // MSb first
      uint8_t mask = 1 << (n - 1);
      while (n-- > 1) {
          sendBit(row & mask);
          row <<= 1;
      }
      setSDIN(row & mask);       // last bit is not clocked it, but set statically
      delayMicroseconds(10);
      break;

  default:
      while (n-- > 0) {
          sendBit(row & 1);    // clock in row number bits 0..(n-1)
          row >>= 1;
      }
      setSDIN(0); // SDIN = 0
      delayMicroseconds(10);
      break;
  }
}


static void delayPrecise(uint16_t msec) {
  if(msec < 16)
    delayMicroseconds(msec * 1000L); // get a better precision: delay(1) can actually be a delay(0)
  else
    delay(msec);
}

// pulse STB pin low for some milliseconds 
static void strobe(unsigned short msec)
{
  delayMicroseconds(20L);
  setSTB(0);
  delayPrecise(msec);
  setSTB(1);
  delayPrecise(msec);
}

// 16V8, 20V8 RA0-5 = row address, strobe.
// 22V10 RA0-5 = 0, send row address (6 bits), strobe.
static void strobeRow(char row)
{
  switch(gal) {
    case GAL16V8:
    case GAL20V8:
    case ATF16V8B:
      setRow(row);         // set RA0-5 to row number
      strobe(2);           // pulse /STB for 2ms
      break;
    case GAL22V10:
    case ATF22V10B:
    case ATF22V10C:
    case ATF750C:
      setRow(0);           // set RA0-5 low
      delayMicroseconds(10);
      sendAddress(row);  // send row number (6 bits)
      delayMicroseconds(10);
      setSTB(0);
      delayMicroseconds(10);
      setSTB(1);           // pulse /STB
      delayMicroseconds(10);
      setSDIN(0);          // SDIN low
      delayMicroseconds(10);
      break;
   }
}

static void strobeConfigRow(char row)
{
  switch(gal) {
    case ATF750C:
      setRow(0);           // set RA0-5 low
      delayMicroseconds(10);
      setRow(galinfo[gal].cfgrow);
      delayMicroseconds(15);

      sendAddress(row);  // send row number (6 bits)
      setSDIN(1);          // SDIN high
      delayMicroseconds(10);

      setSTB(0);
      delayMicroseconds(10);
      setSTB(1);           // pulse /STB

      delayMicroseconds(70);
      break;
   }
}


// read PES: programmer electronic signature (ATF = text string, others = Vendor/Vpp/timing)
static void readPes(void) {
  unsigned short bitmask;
  short byteIndex;

#ifdef DEBUG_PES 
  Serial.print(F("testing gal "));
  Serial.print(gal, DEC);
  Serial.println();
#endif
  turnOn(READPES);

  strobeRow(galinfo[gal].pesrow);


  for(byteIndex = 0; byteIndex < galinfo[gal].pesbytes; byteIndex++) {
    unsigned char value = 0;
    
    for (bitmask = 0x1; bitmask <= 0x80; bitmask <<= 1) {
      if (receiveBit()) {
       value |= bitmask;
      }
    }
    pes[byteIndex] = value;
  }

  turnOff();
}


static unsigned char getDuration(unsigned char index) {
  switch (index) {
    case 0: return 1;
    case 1: return 2;
    case 2: return 5;
    case 3: return 10;
    case 4: return 20;
    case 5: return 30;
    case 6: return 40;
    case 7: return 50;
    case 8: return 60;
    case 9: return 70;
    case 10: return 80;
    case 11: return 90;
    case 12: return 100;
    case 13: return 200;
    default: return 0;
  }
}

void parsePes(char type) {
  unsigned char algo;

  if (UNKNOWN == type) {
    type = gal; 
  }
  
  switch (type) {
    case ATF16V8B:
    case ATF22V10B:
    case ATF22V10C:
    case ATF750C:
        progtime = 10;
        erasetime = 100;
        vpp = 48;    /* 12.0V */
    break;
    default:
        algo = pes[1] & 0x0F;
        if (algo == 5) {
            erasetime = (25 << ((pes[4] >> 2) &7)) / 2;
            progtime = getDuration(((((unsigned short)pes[5] << 8)| pes[4]) >> 5) & 15);
            vpp = 2 * ((pes[5] >> 1) & 31) + 20;
        }
        else switch(gal) {
        case GAL16V8:
        case GAL20V8:
          erasetime=100;
          switch(algo) {
            case 0:
                vpp = 63; // 15.75V
                progtime = 100;
                break;
            case 1:
                vpp = 63; // 15.75V
                progtime = 80;
                break;
            case 2:
                vpp = 66; // 16.5V
                progtime = 10;
                break;
            case 3:
                vpp = (pes[3] == NATIONAL) ? 60 : 58; // 15.0V or 14.5V
                progtime = 40;
                break;
            case 4:
                vpp = 56; // 14V
                progtime = 100;
                break;
            }
            break;
        default:
            erasetime = (pes[3] == NATIONAL) ? 50 : 100;
            switch(algo) {
            case 0:
                vpp = 66; // 16.5V
                progtime = 10;
                break;
            case 1:
                vpp = 63; // 15.75V
                progtime = 100;
                break;
            case 2:
                vpp = (pes[3] == NATIONAL) ? 60 : 58; // 15.0V or 14.5V
                progtime = 40;
                break;
            case 3:
                vpp = 56; // 14V
                progtime = 100;
                break;
            }
        }
    }

    //Afterburnes seems to work with programming voltages reduced by 2V
    vpp -= 8; // -2V
}


// print PES information
void printPes(char type) {
  
  Serial.print(F("PES info: "));
  //voltage
  if (pes[3] == ATMEL16 || pes[3] == ATMEL22) {
     //Serial.print("  ");
  } else {
    if (pes[1] & 0x10) {
      Serial.print(F("3.3V "));
    } else {
      Serial.print(F("5V "));
    }
  }

  //manufacturer
  switch (pes[3]) {
    case LATTICE:    Serial.print(F("Lattice ")); break;
    case NATIONAL:   Serial.print(F("National ")); break;
    case SGSTHOMSON: Serial.print(F("ST Microsystems ")); break;
    case ATMEL16:
    case ATMEL22:    Serial.print(F("Atmel ")); break;
    default:         Serial.print(F("Unknown GAL, "));
  }

  // GAL type
  switch (type) {
    case GAL16V8: Serial.print(F("GAL16V8 ")); break;
    case GAL20V8: Serial.print(F("GAL20V8 ")); break;
    case GAL22V10: Serial.print(F("GAL20V10 ")); break;
    case ATF16V8B: Serial.print(F("ATF16V8B ")); break;
    case ATF22V10B: Serial.print(F("ATF22V10B ")); break;
    case ATF22V10C: Serial.print(F("ATF22V10C ")); break;
    case ATF750C: Serial.print(F("ATF750C ")); break;
  }

  //programming info
  if (UNKNOWN != type) {
    Serial.print(F(" VPP="));
    Serial.print(vpp >> 2, DEC);
    Serial.print(F("."));
    Serial.print((vpp & 3) * 25, DEC);
    Serial.print(F(" Timing: prog="));
    Serial.print(progtime, DEC);
    Serial.print(F(" erase="));
    Serial.print(erasetime / 4, DEC);
  } else {
    Serial.print(F(" try VPP=10..14 in 1V steps"));
  }
  
  Serial.println();
}

// sets a fuse bit on particular position
// expects that the fusemap was cleared (set to zero) beforehand
static void setFuseBit(unsigned short bitPos) {
    fusemap[bitPos >> 3] |= (1 << (bitPos & 7));
}

// gets a fuse bit from specific fuse position
static char getFuseBit(unsigned short bitPos) {
  return (fusemap[bitPos >> 3] & (1 << (bitPos & 7))) ? 1 : 0;
}

// range begins at the beginning of the buffer
static void readGalFuseMapRange(const unsigned char* cfgArray, char useDelay, char doDiscardBits) {
  unsigned short row, bit;
  unsigned short addr;

  if(rangeMemType == range_mem_type::fuses) {
    // read fuse rows
    for(row = rangeStartRow; row < (rangeStartRow + rangeRowCount); row++) {
      strobeRow(row); 
      for(bit = 0; bit < galinfo[gal].bits; bit++) {
        // check the received bit is 1 and if so then set the fuse map
        if (receiveBit()) {
          addr = galinfo[gal].bits * (row - rangeStartRow) + bit;
          setFuseBit(addr);
        }
      }
      if (useDelay) {
        delayPrecise(useDelay); // TODO: shouldn't this be between strobeRow() and receiveBit()?
      }
    }
  }

  if(rangeMemType == range_mem_type::ues) {
    // read UES
    strobeRow(galinfo[gal].uesrow);
    if (doDiscardBits) {
      discardBits(doDiscardBits);
    }
    for(bit = 0; bit < galinfo[gal].uesbytes * 8; bit++) {
      if (receiveBit()) {
        addr = bit;
        setFuseBit(addr);
      }
    }
    if (useDelay) {
      delayPrecise(useDelay);
    }
  }

  if(rangeMemType == range_mem_type::cfg) {
    // read CFG
    if (galinfo[gal].cfgmethod == CFG_STROBE_ROW) {
      strobeRow(galinfo[gal].cfgrow);
    } else {
      setRow(galinfo[gal].cfgrow);
      strobe(1);
    }
    for(bit = 0; bit < galinfo[gal].cfgbits; bit++) {
      if(gal == ATF750C) // TODO: replace by flag in config
        discardBits(galinfo[gal].bits - galinfo[gal].cfgbits);

      if (receiveBit()) {
        setFuseBit(cfgArray[bit]);
      }
    }
  }
}

// generic fuse-map reading, fuse-map bits are stored in fusemap array
static void readGalFuseMap(const unsigned char* cfgArray, char useDelay, char doDiscardBits) {
  unsigned short cfgAddr = galinfo[gal].cfgbase;
  unsigned short row, bit;
  unsigned short addr;

  // read fuse rows
  for(row = 0; row < galinfo[gal].rows; row++) {
    strobeRow(row);
    for(bit = 0; bit < galinfo[gal].bits; bit++) {
      // check the received bit is 1 and if so then set the fuse map
      if (receiveBit()) {
        addr = galinfo[gal].rows;
        addr *= bit;
        addr += row;
        setFuseBit(addr);
      }
    }
    if (useDelay) {
      delayPrecise(useDelay); // TODO: shouldn't this be between strobeRow() and receiveBit()?
    }
  }

   // read UES
  strobeRow(galinfo[gal].uesrow);
  if (doDiscardBits) {
    discardBits(doDiscardBits);
  }
  for(bit = 0; bit < galinfo[gal].uesbytes * 8; bit++) {
    if (receiveBit()) {
      addr = galinfo[gal].uesfuse;
      addr += bit;
      setFuseBit(addr);
    }
  }
  if (useDelay) {
    delayPrecise(useDelay);
  }

  // read CFG
  if (galinfo[gal].cfgmethod == CFG_STROBE_ROW) {
    strobeRow(galinfo[gal].cfgrow);
  } else {
    setRow(galinfo[gal].cfgrow);
    strobe(1);
  }
  for(bit = 0; bit < galinfo[gal].cfgbits; bit++) {
    if (receiveBit()) {
      setFuseBit(cfgAddr + cfgArray[bit]);
    }
  }
}

// generic fuse-map verification, fuse map bits are compared against read bits
static unsigned short verifyGalFuseMapRange(const unsigned char* cfgArray, char useDelay, char doDiscardBits) {
  unsigned short row, bit;
  unsigned short addr;
  char fuseBit;   // fuse bit received from GAL
  char mapBit;    // fuse bit stored in RAM
  unsigned short errors = 0;

  if(rangeMemType == range_mem_type::fuses) {
    // read fuse rows
    for(row = rangeStartRow; row < (rangeStartRow + rangeRowCount); row++) {
      strobeRow(row);
      for(bit = 0; bit < galinfo[gal].bits; bit++) {
        addr = galinfo[gal].bits * (row - rangeStartRow) + bit;
        mapBit = getFuseBit(addr);
        fuseBit = receiveBit();
        if (mapBit != fuseBit) {
  #ifdef DEBUG_VERIFY
          Serial.print(F("f a="));
          Serial.print(bit, DEC);
          Serial.print(F(" "));
          Serial.print(addr, DEC);
          Serial.print(F(" "));
          Serial.print(mapBit, DEC);
          Serial.println(fuseBit, DEC);
  #endif
          errors++;
        }
      }
      if (useDelay) {
        delayPrecise(useDelay);
      }
    }
  }

  if(rangeMemType == range_mem_type::ues) {
    // read UES
    strobeRow(galinfo[gal].uesrow);
    if (doDiscardBits) {
      discardBits(doDiscardBits);
    }
    for(bit = 0; bit < galinfo[gal].uesbytes * 8; bit++) {
      addr = rangeStartRow + bit;
      mapBit = getFuseBit(addr);
      fuseBit = receiveBit();
      if (mapBit != fuseBit) {
  #ifdef DEBUG_VERIFY
        Serial.print(F("U a="));
        Serial.print(bit, DEC);
        Serial.print(F(" "));
        Serial.print(addr, DEC);
        Serial.print(F(" "));
        Serial.print(mapBit, DEC);
        Serial.println(fuseBit, DEC);
  #endif
        errors++;
      }
    }
    if (useDelay) {
      delayPrecise(useDelay);
    }
  }

  if(rangeMemType == range_mem_type::cfg) {
    // read CFG
    if (galinfo[gal].cfgmethod == CFG_STROBE_ROW) {
      strobeRow(galinfo[gal].cfgrow);
    } else {
      setRow(galinfo[gal].cfgrow);
      strobe(1);
    }
    if(gal == ATF750C) // TODO: replace by flag in config
      discardBits(galinfo[gal].bits - galinfo[gal].cfgbits);
    for(bit = 0; bit < galinfo[gal].cfgbits; bit++) {
      uint16_t addr = rangeStartRow + cfgArray[bit];
      mapBit = getFuseBit(addr); 
      fuseBit = receiveBit();
      if (mapBit != fuseBit) {
  #ifdef DEBUG_VERIFY
        Serial.print(F("C a="));
        Serial.print(bit, DEC);
        Serial.print(F(" "));
        Serial.print(addr, DEC);
        Serial.print(F(" "));
        Serial.print(mapBit, DEC);
        Serial.println(fuseBit, DEC);
  #endif
        errors++;
      }
    }
  }

  return errors;
}


// generic fuse-map verification, fuse map bits are compared against read bits
static unsigned short verifyGalFuseMap(const unsigned char* cfgArray, char useDelay, char doDiscardBits) {
  unsigned short cfgAddr = galinfo[gal].cfgbase;
  unsigned short row, bit;
  unsigned short addr;
  char fuseBit;   // fuse bit received from GAL
  char mapBit;    // fuse bit stored in RAM
  unsigned short errors = 0;

  // read fuse rows
  for(row = 0; row < galinfo[gal].rows; row++) {
    strobeRow(row);
    for(bit = 0; bit < galinfo[gal].bits; bit++) {
      addr = galinfo[gal].rows;
      addr *= bit;
      addr += row;
      mapBit = getFuseBit(addr);
      fuseBit = receiveBit();
      if (mapBit != fuseBit) {
#ifdef DEBUG_VERIFY
        Serial.print(F("f a="));
        Serial.print(bit, DEC);
        Serial.print(F(" "));
        Serial.print(addr, DEC);
        Serial.print(F(" "));
        Serial.print(mapBit, DEC);
        Serial.println(fuseBit, DEC);
#endif
        errors++;
      }
    }
    if (useDelay) {
      delayPrecise(useDelay);
    }
  }

   // read UES
  strobeRow(galinfo[gal].uesrow);
  if (doDiscardBits) {
    discardBits(doDiscardBits);
  }
  for(bit = 0; bit < galinfo[gal].uesbytes * 8; bit++) {
    addr = galinfo[gal].uesfuse;
    addr += bit;
    mapBit = getFuseBit(addr);
    fuseBit = receiveBit();
    if (mapBit != fuseBit) {
#ifdef DEBUG_VERIFY
      Serial.print(F("U a="));
      Serial.print(bit, DEC);
      Serial.print(F(" "));
      Serial.print(addr, DEC);
      Serial.print(F(" "));
      Serial.print(mapBit, DEC);
      Serial.println(fuseBit, DEC);
#endif
      errors++;
    }
  }
  if (useDelay) {
    delayPrecise(useDelay);
  }

  // read CFG
  if (galinfo[gal].cfgmethod == CFG_STROBE_ROW) {
    strobeRow(galinfo[gal].cfgrow);
  } else {
    setRow(galinfo[gal].cfgrow);
    strobe(1);
  }
  for(bit = 0; bit < galinfo[gal].cfgbits; bit++) {
    uint16_t addr = cfgAddr + cfgArray[bit];
    mapBit = getFuseBit(addr); 
    fuseBit = receiveBit();
    if (mapBit != fuseBit) {
#ifdef DEBUG_VERIFY
      Serial.print(F("C a="));
      Serial.print(bit, DEC);
      Serial.print(F(" "));
      Serial.print(addr, DEC);
      Serial.print(F(" "));
      Serial.print(mapBit, DEC);
      Serial.println(fuseBit, DEC);
#endif
      errors++;
    }
  }
  return errors;
}


// main fuse-map reading and verification function
// READING: reads fuse rows, UES, CFG from GAL and stores into fusemap bit array RAM.
// VERIFY:  reads fuse rows, UES, CFG from GAL and compares with fusemap bit array in RAM.
static void readOrVerifyGal(char verify)
{
  unsigned short i;

  //ensure fusemap is cleared before READ operation, keep it for VERIFY operation.
  if (!verify) {
    for (i = 0; i < MAXFUSES; i++) {
      fusemap[i] = 0;
    }
  }

  turnOn(READGAL);

  switch(gal)
  {
    case GAL16V8:
    case GAL20V8: {
        unsigned char* cfgArray = (unsigned char*) cfgV8;
        if (pes[2] == 0x1A || pes[2] == 0x3A) {
          cfgArray = (unsigned char*) cfgV8AB;
        }
        //read without delay, no discard
        if (verify) {
          i = verifyGalFuseMap(cfgArray, 0, 0);
        } else {
          readGalFuseMap(cfgArray, 0, 0);
        }}
        break;
      
    case ATF16V8B:
        //read without delay, no discard
        if (verify) {
          i = verifyGalFuseMap(galinfo[gal].cfg, 0, 0);
        } else {
          readGalFuseMap(galinfo[gal].cfg, 0, 0);
        }
        break;
      
    case GAL22V10:
    case ATF22V10B:
    //case ATF22V10C:  // TODO: reactivate this variant again and remove below
      //read with delay 1 ms, discard 68 cfg bits on ATF22V10B/C and 107 on ATF750C
      if (verify) {
        i = verifyGalFuseMap(galinfo[gal].cfg, 1,
           (gal == GAL22V10) ? 0 : galinfo[gal].bits - 8 * galinfo[gal].uesbytes);
      } else {
        readGalFuseMap(galinfo[gal].cfg, 1,
          (gal == GAL22V10) ? 0 : galinfo[gal].bits - 8 * galinfo[gal].uesbytes);
      } 
      break;

    case ATF22V10C: // TODO: reactivate the non-Range variant above
    case ATF750C:
      //read with delay 1 ms, discard 107 bits on ATF750C
      if (verify) {
        i = verifyGalFuseMapRange(galinfo[gal].cfg, 1, galinfo[gal].bits - 8 * galinfo[gal].uesbytes);
      } else {
        readGalFuseMapRange(galinfo[gal].cfg, 1, galinfo[gal].bits - 8 * galinfo[gal].uesbytes);
      } 
      break;
  }
  turnOff();

  if (verify && i > 0) {
    Serial.print(F("ER verify failed. Bit errors: "));
    Serial.println(i, DEC);
  }
}

// fuse-map writing function for V8 GAL chips
static void writeGalFuseMapV8(const unsigned char* cfgArray) {
  unsigned short cfgAddr = galinfo[gal].cfgbase;
  unsigned char row, rbit;
  unsigned short addr;

  setPV(1);
  // write fuse rows
  for (row = 0; row < galinfo[gal].rows; row++) {
    setRow(row);
    for(rbit = 0; rbit < galinfo[gal].bits; rbit++) {
      addr = galinfo[gal].rows;
      addr *= rbit;
      addr += row;
      sendBit(getFuseBit(addr));
    }
    strobe(progtime);
  }

  // write UES
  setRow(galinfo[gal].uesrow);
  for (rbit = 0; rbit < galinfo[gal].uesbytes * 8; rbit++) {
    addr = galinfo[gal].uesfuse;
    addr += rbit;
    sendBit(getFuseBit(addr));
  }
  strobe(progtime);

  // write CFG (all ICs use setRow)
  setRow(galinfo[gal].cfgrow);
  for(rbit = 0; rbit < galinfo[gal].cfgbits; rbit++) {
    sendBit(getFuseBit(cfgAddr + cfgArray[rbit]));
  }
  strobe(progtime);
  setPV(0);
}

// fuse-map writing function for V10 GAL chips
static void writeGalFuseMapV10(const unsigned char* cfgArray, char fillUesStart, char useSdin) {
  unsigned short cfgAddr = galinfo[gal].cfgbase;
  unsigned char row, bit;
  unsigned short addr;

  setRow(0); //RA0-5 low
  // write fuse rows
  for (row = 0; row < galinfo[gal].rows; row++) {
    for (bit = 0; bit < galinfo[gal].bits; bit++) {
      addr = galinfo[gal].rows;
      addr *= bit;
      addr += row;
      sendBit(getFuseBit(addr));
    }
    sendAddress(row);
    setPV(1);
    strobe(progtime);
    setPV(0);
  }

  // write UES
  if (fillUesStart) {
    sendBits(galinfo[gal].bits - 8 * galinfo[gal].uesbytes, 1);
  }
  for (bit = 0; bit < galinfo[gal].uesbytes * 8; bit++) {
    addr = galinfo[gal].uesfuse;
    addr += bit;
    sendBit(getFuseBit(addr));
  }
  if (!fillUesStart) {
    sendBits(galinfo[gal].bits - 8 * galinfo[gal].uesbytes, 1);
  }
  sendAddress(galinfo[gal].uesrow);
  setPV(1);
  strobe(progtime);
  setPV(0);
  
  // write CFG
  setRow(galinfo[gal].cfgrow);
  for(bit = 0; bit < galinfo[gal].cfgbits - useSdin; bit++) {
    sendBit(getFuseBit(cfgAddr + cfgArray[bit]));
  }
  if (useSdin) {
    setSDIN(getFuseBit(cfgAddr + cfgArray[galinfo[gal].cfgbits - 1])); // last config bit is power down bit
    // TODO: the logic is wrong, because that would need 20+1 bits for ATF22V10C, but the array only contains 20 values
    // maybe:     setSDIN(getFuseBit(cfgAddr + cfgArray[20]));, but then array must be longer
  }
  setPV(1);
  strobe(progtime);
  setPV(0);

  if (useSdin) {
    // disable power-down feature (JEDEC bit #5892)
    setRow(0);
    sendAddress(59);
    setPV(1);
    strobe(progtime);
    setPV(0);
  }
  
}

// fuse-map writing function for ATF750C chips
// works also for ATF22V10C
static void writeGalFuseMapV750CRange(const unsigned char* cfgArray, char fillUesStart, char useSdin) {
  unsigned short cfgAddr = galinfo[gal].cfgbase;
  unsigned char row, bit;
  unsigned short addr;
	
  if(rangeMemType == range_mem_type::fuses) {
    // write fuse rows
	  setRow(0); //RA0-5 low
    for(row = rangeStartRow; row < (rangeStartRow + rangeRowCount); row++) {
      for (bit = 0; bit < galinfo[gal].bits; bit++) {
        addr = galinfo[gal].bits * (row - rangeStartRow) + bit;
        sendBit(getFuseBit(addr));
      }

      sendAddress(row);      
      setPV(1);
      delayMicroseconds(100);
      strobe(progtime);
      setPV(0);
      delayMicroseconds(100);
    }
  }

  if(rangeMemType == range_mem_type::ues) {
    // write UES
    setRow(0); //RA0-5 low
	  if (fillUesStart) {
      sendBits(galinfo[gal].bits - (8 * galinfo[gal].uesbytes), 1);
    }
    for (bit = 0; bit < (8 * galinfo[gal].uesbytes); bit++) {
      addr = rangeStartRow + bit;
      sendBit(getFuseBit(addr));
    }
    if (!fillUesStart) {
      sendBits(galinfo[gal].bits - (8 * galinfo[gal].uesbytes), 1);
    }

    uint8_t row = galinfo[gal].uesrow;
    sendAddress(row); 
    setPV(1);
    strobe(progtime);
    setPV(0);
	  delayPrecise(progtime);
  }

#if 1
  if(rangeMemType == range_mem_type::cfg) {
    // write CFG
    setRow(galinfo[gal].cfgrow);
    if(gal == ATF750C) {
    //if(0) {
      for(bit = 0; bit < galinfo[gal].cfgbits; bit++) {
	      //sendBit(getFuseBit(rangeStartRow + cfgArray[bit]));
	      //sendBit(bit & 0x01); // TODO: dummy data
	      sendBit(0);
	    }
#if 1
      // shift the cfg bits to the beginning of the row
	    for(; bit < galinfo[gal].bits; bit++) {
	      sendBit(0);
	    }
#endif
    }
    else
    {  	
      for(bit = 0; bit < galinfo[gal].cfgbits - useSdin; bit++) {
	      sendBit(getFuseBit(rangeStartRow + cfgArray[bit]));
	      //sendBit(bit & 0x01); // TODO: dummy data
	    }
	    if (useSdin) {
	      setSDIN(getFuseBit(rangeStartRow + cfgArray[galinfo[gal].cfgbits - 1]));
	    }
  	}
	  setPV(1);
    strobe(progtime);
    setPV(0);
	  delayPrecise(progtime);

    if (useSdin) {
      // disable power-down feature (JEDEC bit #5892)
      setRow(0);


sendBits(galinfo[gal].bits, 0); // TODO: remove; just for testing

      uint8_t row = galinfo[gal].uesrow + 1; // TODO: check if row is correct
      sendAddress(row); 
      setPV(1);
      strobe(progtime);
      setPV(0);
    }
  }
#endif  
}


// main fuse-map writing function
static void writeGal()
{
  unsigned short i;
  unsigned char* cfgArray = (unsigned char*) cfgV8;


  turnOn(WRITEGAL);

  switch(gal)
  {
    case GAL16V8:
    case GAL20V8:
        if (pes[2] == 0x1A || pes[2] == 0x3A) {
          cfgArray = (unsigned char*) cfgV8AB;
        }
        writeGalFuseMapV8(cfgArray); 
        break;
      
    case ATF16V8B:
        writeGalFuseMapV8(cfgV8AB); 
        break;

    case GAL22V10:
    case ATF22V10B:
    //case ATF22V10C: // TODO: reactivate this non-Range variant and remove below
        writeGalFuseMapV10(cfgV10, (gal == GAL22V10) ? 0 : 1, (gal == ATF22V10C) ? 1 : 0);
        break; 

    case ATF22V10C: // TODO: reactivate the non-Range variant above
    case ATF750C:
        writeGalFuseMapV750CRange(cfgV750, 1, 1); // TODO: fix 2xconstant 1
        break; 

  }
  turnOff();
}

// erases fuse-map in the GAL
static void eraseGAL(void)
{
    turnOn(ERASEGAL);
    
    setPV(1);
    setRow(galinfo[gal].eraserow);
    if (gal == GAL16V8 || gal == ATF16V8B || gal==GAL20V8) {
        sendBit(1);
    }
    strobe(erasetime);
    setPV(0);
    turnOff();
}


static char checkGalTypeViaPes(void)
{
    char type = UNKNOWN;

#ifdef DEBUG_PES
    char i;
    Serial.println(F("PES raw bytes:"));
    for (i = 0; i < 10; i++) {
      printFormatedNumberHex2(pes[i]);
      Serial.print(F(" "));
    }
    Serial.println();
    for (i = 0; i < 10; i++) {
      Serial.write(pes[i]);
    }
    Serial.println();
#endif

    if (pes[7] == 'F' && pes[6]== '2' && pes[5]== '2' && pes[4]== 'V' && pes[3]== '1' && pes[2]=='0') {
       if (pes[1] == 'B') {
           type = ATF22V10B;
       } else {
           type = ATF22V10C;
       }
    }
    else if (pes[8] == 'F' && pes[7] == 'V' && pes[6] == '7' && pes[5] == '5' && pes[4] == '0' && pes[3] =='C') {
      // complete string at beginning of row 127: "300C057VF100"
      type = ATF750C;
    }
    else if (pes[6] == 'F' && pes[5] == '1' && pes[4]== '6' && pes[3] == 'V' && pes[2]=='8') {
       type = ATF16V8B;
    }
    else if (pes[2] != 0x00 && pes[2] != 0xFF) {
       for (type = (sizeof(galinfo) / sizeof(galinfo[0])) - 1; type; type--) {
           if (pes[2] == galinfo[type].id0 || pes[2] == galinfo[type].id1) break;
       }
    }

    return type;
}

// checks whether gal type corresponds to PES information on the IC
// note: PES must be read beforehand
static char testProperGAL(void)
{
    char type = checkGalTypeViaPes();

    if (type == 0) {
       //Unknown or illegal PES,
       goto error;
    }
    else if (type != gal) {
      //PES indicates a different GAL type than selected. Change to detected GAL type?
      goto error;
    }

    return 1;

error:
    Serial.print(F("ER unknown or wrong GAL type (check Power ON), "));
    Serial.print(type, DEC);
    Serial.print(F("!="));
    Serial.println(gal, DEC);
    return 0;
}

// prints a hexadecimal number - 2 digits with a leading zero
static void printFormatedNumberHex2(unsigned char num) {
  if (num < 16) {
    Serial.print(F("0"));
  }
  Serial.print(num, HEX);  
}

// prints a hexadecimal number - 4 digits with a leading zero
static void printFormatedNumberHex4(unsigned short num) {
  if (num < 0x10) {
    Serial.print(F("000"));
  } else
  if (num < 0x100) {
    Serial.print(F("00"));
  } else
  if (num < 0x1000) {
    Serial.print(F("0"));
  }
  Serial.print(num, HEX);  
}

// prints a decimal number - 4 digits with a leading zero
static void printFormatedNumberDec4(unsigned short num) {
  if (num < 1) {
    Serial.print(F("0000"));
    return;
  }

  if (num < 10) {
    Serial.print(F("000"));
  } else
  if (num < 100) {
    Serial.print(F("00"));
  } else
  if (num < 1000) {
    Serial.print(F("0"));
  }
  Serial.print(num, DEC);  
}

// adds a formated decimal number with a leading zero to a line buffer at position 'i'
static unsigned char addFormatedNumberDec4(unsigned short num, unsigned char i) {
  char cnt = 3;

  while (cnt >= 0) {
    line[i + cnt] = '0' + (num % 10);
    num /= 10;
    cnt--;
  }
  return i + 4;
}

// calculates fuse-map checksum and returns it
static unsigned short checkSum(unsigned short n)
{
    unsigned short c, e, i;
    unsigned long a;

    c = e= 0;
    a = 0;
    for (i = 0; i < n; i++) {
        e++;
        if (e == 9) {
          e = 1;
          a += c;
          c = 0;
        }
        c >>= 1;
        if (getFuseBit(i)) {
          c += 0x80;
        }
    }
    return (unsigned short)((c >> (8 - e)) + a);
}

// this prints not really JEDEC format, the tranpose of the fuse area is NOT done
// here (it's not possible because only a part of the data is in the buffer)
// and the data is printed in hex so the length is rounded up to be a multiple of 8 bits
// each line is prefixed by row number in hex (2 digits)
static void printJedecRange()
{
  uint16_t row, bit, addr;
  uint16_t csBitCount = 0;

  if(rangeMemType == range_mem_type::fuses) {
    csBitCount = galinfo[gal].bits * rangeRowCount;
    for(row = rangeStartRow; row < (rangeStartRow + rangeRowCount); row++) {
      printFormatedNumberHex2(row);
      Serial.write(' ');
      for(bit = 0; bit < ((galinfo[gal].bits + 7) & 0xfff8); bit += 8) {
        addr = galinfo[gal].bits * (row - rangeStartRow) + bit;
        uint8_t b = 0;
        for(uint8_t i = 0; (i < 8) && ((bit + i) < galinfo[gal].bits); i++) {
          b |= getFuseBit(addr + i) ? (1 << i) : 0;
        }
        printFormatedNumberHex2(b);
      }
      Serial.write('\r');
    }
  }

  if(rangeMemType == range_mem_type::ues) {
    csBitCount = galinfo[gal].uesbytes * 8;
    row = 0;
    printFormatedNumberHex2(row);
    Serial.write(' ');
    for(bit = 0; bit < (galinfo[gal].uesbytes * 8); bit += 8) {
      addr = bit;
      uint8_t b = 0;
      for(uint8_t i = 0; i < 8; i++) {
        b |= getFuseBit(addr + i) ? (1 << i) : 0;
      }
      printFormatedNumberHex2(b);
    }
    Serial.write('\r');
  }

  if(rangeMemType == range_mem_type::cfg) {
    csBitCount = galinfo[gal].cfgbits;
    row = 0;
    printFormatedNumberHex2(row);
    Serial.write(' ');
    for(bit = 0; bit < ((galinfo[gal].cfgbits + 7) & 0xfff8); bit += 8) {
      addr = bit;
      uint8_t b = 0;
      for(uint8_t i = 0; i < 8; i++) {
        b |= getFuseBit(addr + i) ? (1 << i) : 0;
      }
      printFormatedNumberHex2(b);
    }
    Serial.write('\r');
  }
  
  uint16_t check = checkSum(csBitCount);
  printFormatedNumberHex4(check);
  Serial.write('\r');
}

// prints the contents of fuse-map array in the form of JEDEC text file
static void printJedec()
{
    unsigned short i, j, k, n;
    unsigned char unused, start;

    Serial.print(F("JEDEC file for "));
    Serial.println(galinfo[gal].name);
    Serial.print(F("*QP")); Serial.print(galinfo[gal].pins, DEC);
    Serial.print(F("*QF")); Serial.print(galinfo[gal].fuses, DEC);
    Serial.println(F("*QV0*F0*G0*X0*"));
    
    for( i = k = 0; i < galinfo[gal].bits; i++) {
        unused = 1;
        n = 0;
        line[n++] = 'L';
        n = addFormatedNumberDec4(k, n);
        line[n++] = ' ';
        for(j= 0; j < galinfo[gal].rows; j++, k++) {
           if (getFuseBit(k)) {
              unused = 0;
              line[n++] = '1';
           } else {
              line[n++] = '0';
           }
        }
        line[n++] = '*';
        line[n++] = 0;
        if (!unused) {
          Serial.println(line);
        }
    }

    if( k < galinfo[gal].uesfuse) {
        unused = 1;
        n = 0;
        line[n++] = 'L';
        n = addFormatedNumberDec4(k, n);
        line[n++] = ' ';
        
        while(k < galinfo[gal].uesfuse) {
           if (getFuseBit(k)) {
              unused = 0;
              line[n++] = '1';
           } else {
              line[n++] = '0';
           }
           k++;
        }
        line[n++] = '*';
        line[n++] = 0;
        if (!unused) {
          Serial.println(line);
        }
    }
    line[0] = 0;


    Serial.print(F("N UES"));
    for (j = 0;j < galinfo[gal].uesbytes; j++) {
        n = 0;
        for (i = 0; i < 8; i++) {
            if (getFuseBit(k + 8 * j + i)) {
                if ((gal == ATF22V10C) || (gal == ATF750C)) {
                    n |= 1 << (7 - i);  // big-endian
                }
                else {
                    n |= 1 << i;     // little-endian
                }
            }
        }
        Serial.print(F(" "));
        printFormatedNumberHex2(n);  
    }
    Serial.println(F("*"));

    Serial.print(F("L"));
    printFormatedNumberDec4(k);
    Serial.print(F(" "));

    for(j = 0; j < 8 * galinfo[gal].uesbytes; j++) {
      if (getFuseBit(k++)) {
         Serial.print(F("1"));
      } else {
         Serial.print(F("0"));
      }
    }
    Serial.println(F("*"));


    if (k < galinfo[gal].fuses) {
      Serial.print(F("L"));
      printFormatedNumberDec4(k);
      Serial.print(F(" "));

      while( k < galinfo[gal].fuses) {
        if (getFuseBit(k++)) {
           Serial.print(F("1"));
        } else {
           Serial.print(F("0"));
        }
      }
      Serial.println(F("*"));
    }

    Serial.print(F("N PES"));
    for(i = 0; i < galinfo[gal].pesbytes; i++) {
        Serial.print(F(" "));
        printFormatedNumberHex2(pes[i]);  
    }
    Serial.println(F("*"));
    Serial.print(F("C"));
    printFormatedNumberHex4(checkSum(galinfo[gal].fuses));
    Serial.println();
    Serial.println(F("*"));
}


// helper print function to save RAM space
static void printNoFusesError() {
  Serial.println(F("ER fuse map not uploaded"));
}

static void testVoltage(int seconds) {
  int i;
  setVPP(1);
  for (i = 0 ; i < seconds; i++) {
    delayPrecise(1000);
  }
  setVPP(0);
}


// returns 1 if type check if OK, 0 if gal type does not match the type read from PES
static char doTypeCheck(void) {
  
  if (0 == typeCheck) {
    return 1; // no need to do type check
  }
  readPes();
  parsePes(UNKNOWN);
  return testProperGAL();
}

// Arduino main loop
void loop() {
    // read a command from serial terminal or COMMAND_NONE if nothing or incomplete cmd is received from serial
    char command = handleTerminalCommands();

    // any unexpected input when uploading fuse map terminates the upload process
    if (isUploading && command != COMMAND_UTX && command != COMMAND_NONE) {
      Serial.println(F("ER upload aborted"));
      isUploading = 0;
      lineIndex = 0;
    }

    // handle commands received from the serial terminal
    switch (command) {

#ifdef TEST750
      // read fuse-map from the GAL and print it in the JEDEC form
      case 'o' : {
      	testRead();
      } break;
#endif
      
      // print some help
      case COMMAND_HELP: {
        printHelp(1);
      } break;

      case COMMAND_IDENTIFY_PROGRAMMER : {
        printHelp(0);
      } break;

      // verify fuse-map bits and bits read from the GAL chip
      case COMMAND_VERIFY_FUSES_RANGE: {
        parseRangeOptions();
        if (mapUploaded) {
          if (doTypeCheck()) {
            readOrVerifyGal(1); //just verify, do not overwrite fusemap
          }
        } else {
          printNoFusesError();
        }
      } break;

      // verify fuse-map bits and bits read from the GAL chip
      case COMMAND_VERIFY_FUSES: {
        clearRangeOptions();
        if (mapUploaded) {
          if (doTypeCheck()) {
            readOrVerifyGal(1); //just verify, do not overwrite fusemap
          }
        } else {
          printNoFusesError();
        }
      } break;

      // handle upload command - start the download of fuse-map
      case COMMAND_UPLOAD: {
        short i;
        // clean fuses
        for (i = 0; i < MAXFUSES; i++) {
          fusemap[i] = 0;
        }
        isUploading = 1;
        uploadError = 0;
      } break;

      // command of the upload protocol
      case COMMAND_UTX : {
        parseUploadLine();
      } break;

      // read and print the PES
      case COMMAND_READ_PES : {
        char type;
        readPes();
        type = checkGalTypeViaPes();
        parsePes(type);
        printPes(type);
      } break;

      case COMMAND_READ_FUSES_RANGE : {
        parseRangeOptions();
        if (doTypeCheck()) {
          readOrVerifyGal(0); //just read, no verification
          printJedecRange();
        }
      } break;

      // read fuse-map from the GAL and print it in the JEDEC form
      case COMMAND_READ_FUSES : {
        clearRangeOptions();
        if (doTypeCheck()) {
          readOrVerifyGal(0); //just read, no verification
          printJedec();
        }
      } break;

      case COMMAND_WRITE_FUSES_RANGE : {
        parseRangeOptions();
        if (mapUploaded) {
          if (doTypeCheck()) {
            writeGal();
            //TODO security
          }
        } else {
          printNoFusesError();
        }
      } break;

      // write current fuse-map to the GAL chip
      case COMMAND_WRITE_FUSES : {
        clearRangeOptions();
        if (mapUploaded) {
          if (doTypeCheck()) {
            writeGal();
            //TODO security
          }
        } else {
          printNoFusesError();
        }
      } break;

      // erases the fuse-map on the GAL chip
      case COMMAND_ERASE_GAL: {
        if (doTypeCheck()) {
          eraseGAL();
        }
      } break;

      // toggles terminal echo
      case COMMAND_ECHO : {
        echoEnabled = 1 - echoEnabled;
      } break;

      case COMMAND_TEST_VOLTAGE : {
        testVoltage(20);
      } break;

      case COMMAND_SET_GAL_TYPE : {
        char type = line[1] - '0';
        if (type >= 1 && type < LAST_GAL_TYPE) {
          gal = (GALTYPE) type;
        } else {
          Serial.println(F("ER Unknown gal type"));
        }
      } break;
      case COMMAND_ENABLE_CHECK_TYPE: {
        typeCheck = 1;
      } break;
      case COMMAND_DISABLE_CHECK_TYPE: {
        typeCheck = 0;
      } break;
      default: {
        if (command != COMMAND_NONE) {
          Serial.print(F("ER Unknown command: '"));
          Serial.write(command);
          Serial.print(F("' / "));
          Serial.println(line);
        }
      }
    }

    // display prompt character - important for the PC program to check that Arduino
    // finished the desired operation
    if (command != COMMAND_NONE) {
      Serial.println(F(">"));
    }

    // and that's it!
}
