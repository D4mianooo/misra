/******************************************************************************
 *
 * Copyright:
 *    (C) 2000 - 2007 Embedded Artists AB
 *
 * Description:
 *    Main program for LPC2148 Education Board test program
 *
 *****************************************************************************/

#include "pre_emptive_os/api/osapi.h"
#include "pre_emptive_os/api/general.h"
#include <printf_P.h>
#include <ea_init.h>
#include <lpc2xxx.h>
#include <consol.h>
#include "i2c.h"
#include "adc.h"
#include "lcd.h"
#include "pca9532.h"
#include "ea_97x60c.h"
#include "irq/irq_handler.h"
#include "VIC.h"
#include "timer.h"
#include "startup/general.h"

#define PROC1_STACK_SIZE 1024
#define INIT_STACK_SIZE  400

// Display string length constants
#define TIME_STR_SIZE 10
#define TEMP_STR_SIZE 5

// Clock sequence
#define SEQ_LEN 8

// Button debounce
#define DEBOUNCE_TIME_MS 50

// Termomether
#define TEMP_ADDR 0x4f

#define _BIT(n)                   (1<<(n))
#define _PIN(port,bit)            ((IOPIN##port & _BIT(bit)) != 0)

// Process
volatile tU32 msClock;
volatile tU8 killProc1 = FALSE;
static tU8 proc1Stack[PROC1_STACK_SIZE];
static tU8 initStack[INIT_STACK_SIZE];
static tU8 pid1;

// RTC Time
int hour = 17;
int minute = 30;
int sec = 5;

// Alarm time
tU16 alarmHour = 17;
tU16 alarmMin = 30;
tU16 alarmSec = 5;

// Alarm time temporary
tU16 alarmHourT = 0;
tU16 alarmMinT = 0;
tU16 alarmSecT = 0;
tU16 checksum = 0;
tU16 isNewEpromValue = 0;

// Clock sound flag
int playSound = 1;

// Process
static void proc1(void* arg);
static void initProc(void* arg);

// Button press P1.20 - P1.23
static tU16 btns[SEQ_LEN] = {0,0,0,0,0,0,0,0};
static int btnsIdx = 0;
static tU16 btnsState[4] = {0,0,0,0};
static int lastBtnTime[4] = {0,0,0,0};

// Sequence
static int seqCounter = 0;
static tU16 seq[SEQ_LEN] = {16, 19, 18, 17, 16, 18, 16, 19};

// LED counter
static unsigned int ledStartTime = 0;
static int ledOn = 0;
static unsigned int ledStartTimeRGB = 0;
static int ledOnRGB = 0;

// Motor counter
static unsigned int motorStartTime = 0;
static int motorState = 0;

static int seqWin = 0;

// Time change
static int setTimeMode = 0;

/*!
 * @brief Get a pseudo random number from a given seed
 *
 * @param seed A seed value.
 *
 * @returns Pseudo random value.
 */
unsigned int simpleRand(int seed) {
    seed = (seed * 1103515245 + 12345) & 0x7FFFFFFF;
    return (unsigned int)(seed >> 16) & 0x7FFF;
}

/*!
 * @brief Flashes the RGB led every 250 ms.
 *
 * @side effects:
 * 	When the function finishes the LED is unpredictable state.
 */
void ledFlashRGB() {
	unsigned int now = T0TC;

	if(ledOnRGB == 0) {
		if((now - ledStartTimeRGB) >= 250) {
			ledOnRGB = 1;
			ledRedOn();
			ledGreenOn();
			ledStartTimeRGB = now;
		}
	} else {
		if((now - ledStartTimeRGB) >= 250) {
			ledOnRGB = 0;
			ledRedOn();
			ledGreenOff();
			ledStartTimeRGB = now;
		}
	}
}

/*!
 * @brief Validates user sequence against the generalated sequence.
 * 		  Sets the P1.16-P1.19 LED state based on generated sequence every second.
 * 		  Turns off the P1.16-P1.19 LEDs when the sequence is valid.
 */
void sequenceCheck() {
	// Compare button press with sequence
	int btn_i = 0;
	for(btn_i = 16; btn_i <= 19; btn_i++) {
		checkCode(btn_i);
	}

	// Check is user sequence matches clock sequence
	int code_i = 0;
	int isValid = 1;
	for(code_i = 0; code_i < SEQ_LEN; code_i++) {
		if(seq[code_i] != btns[code_i]) {
			isValid = 0;
		}
	}

	// Reset
	if(isValid == 1 && seqWin == 0) {
		T1TCR = 0;
		lcdClrscr();
		IOSET1 = (1 << 16);
		IOSET1 = (1 << 17);
		IOSET1 = (1 << 18);
		IOSET1 = (1 << 19);
		seqWin = 1;
	}

	unsigned int now = T0TC;

	if (!ledOn) {
		if (seqCounter < SEQ_LEN && seqCounter >= 0) {
			IOCLR1 = (1 << seq[seqCounter]);
		}
		else if (seqCounter == SEQ_LEN) {
			IOCLR1 = (1 << 16);
			IOCLR1 = (1 << 17);
			IOCLR1 = (1 << 18);
			IOCLR1 = (1 << 19);
		}

		ledStartTime = now;
		ledOn = 1;
	} else {
		if ((now - ledStartTime) >= 1000) {
			if(seqCounter < SEQ_LEN && seqCounter >= 0) {
				IOSET1 = (1 << seq[seqCounter]);
			} else if (seqCounter == SEQ_LEN) {
				IOSET1 = (1 << 16);
				IOSET1 = (1 << 17);
				IOSET1 = (1 << 18);
				IOSET1 = (1 << 19);
				seqCounter = -1;
			}

			seqCounter++;
			ledOn = 0;
		}
	}
}

/*!
 * @brief Generate a random sequence.
 * 		  Seed value is read from the temperature sensor.
 *
 * @param seq A pointer to a sequence array
 * @param len The lenght of the sequence to be generated
 * @param seed A seed for the pseudo random generator
 */
void generateRandomSequence(tU16* seq, int len, int seed) {
    tU8 buf[2];
    tU16 values[4] = {16, 17, 18, 19};

	// Read temp for random seed
    i2cRead(((TEMP_ADDR) << 1) | 1, buf, 2);
    seed = (buf[0] << 8) | buf[1];

	int i = 0;
    for (i = 0; i < len; i++) {
        int idx;
        tU16 newVal;

        do {
            seed = simpleRand(seed);
            idx = seed % 4;
            newVal = values[idx];
        } while (i > 0 && newVal == seq[i - 1]);

        seq[i] = newVal;
    }
}

/*!
 * @brief Check whether the button is pressed.
 * 		  If the pressed button is correct in the sequence display '*',
 * 		  else clear screen and reset sequence.
 *
 * @param btn Button to be checked.
 *
 * @side effects:
 * 	The display will flashes as it is refreshing.
 */
void checkCode(int btn) {
    int btnIdx = btn - 16;
    unsigned int now = T0TC;

    if ((IOPIN1 & (1 << (btn + 4))) == 0) {
        if (btnsState[btnIdx] != 0 || (now - lastBtnTime[btnIdx]) < DEBOUNCE_TIME_MS)
            return;

        lastBtnTime[btnIdx] = now;
        btnsState[btnIdx] = 1;

        btns[btnsIdx] = btn;

        if (seq[btnsIdx] == btns[btnsIdx]) {
            lcdGotoxy(32 + btnsIdx * 8, 42);
            lcdPutchar('*');
            btnsIdx++;
        } else {
            lcdClrscr();
            int i = 0;
            for (i = 0; i < SEQ_LEN; i++) {
                btns[i] = 0;
            }
            btnsIdx = 0;
        }
    } else {
        btnsState[btnIdx] = 0;
    }
}

/*!
 * @brief Inits PWM to control DC motor for vibrations.
 */
void motorStart() {
	PINSEL0 &= ~(3 << 0);
	PINSEL0 |=  (2 << 0);

	PWMMCR = (1 << 1);
	PWMTCR = 0x00;
	PWMPR = 100000;

	PWMMR0 = 1000;
	PWMMR1 = 200;

	PWMLER = (1 << 0) | (1 << 1);
	PWMPCR = (1 << 9);
	PWMTCR = (1 << 0) | (1 << 3);
}

/*!
 * @brief Sets PWM duty cycle to turn off the motor
 */
void motorStop() {
    PWMMR1 = 1000;
    PWMLER = (1 << 1);
}

/*!
 * @brief Turns on the red component for the RGB led.
 */
void ledRedOn() {
    PINSEL1 &= ~0x00030000;
    IODIR0 |= (1 << 17);
    IOCLR0 = (1 << 17);
}

/*!
 * @brief Turns off the red component for the RGB led.
 */
void ledRedOff() {
    IOSET0 = (1 << 17);
}

/*!
 * @brief Turns on the green component for the RGB led.
 */
void ledGreenOn() {
    PINSEL1 &= ~0x00300000;
    IODIR0 |= (1 << 21);
    IOCLR0 = (1 << 21);
}

/*!
 * @brief Turns off the green component for the RGB led.
 */
void ledGreenOff() {
    IOSET0 = (1 << 21);
}

/*!
 * @brief Calculate a XOR checksum for 3 given 16-bit unsigned integers
 *
 * @param a The first value
 * @param b The second value
 * @param c The third value
 *
 * @returns A 16-bit unsigned integer checksum
 */
tU16 checksum3_xor(tU16 a, tU16 b, tU16 c)
{
    return a ^ b ^ c;
}

/*!
 * @brief Write 3 16-bit unsigned integers to EPROM
 * 		  Caclulates the checksum for the data to be inserted.
 * 		  Turns on the blue LED on to signal write finish.
 *
 * @param a The first value
 * @param b The second value
 * @param c The third value
 *
 * @side effects:
 * 	The values can be written incorrectly.
 */
void writeDataWithChecksum(tU16 a, tU16 b, tU16 c)
{
    checksum = checksum3_xor(a, b, c);

    tU8 buffer[6];
    buffer[0] = a >> 8;
    buffer[1] = a & 0xFF;
    buffer[2] = b >> 8;
    buffer[3] = b & 0xFF;
    buffer[4] = c >> 8;
    buffer[5] = c & 0xFF;

    eepromWrite(0x0000, buffer, 6);
    eepromPoll();

    ledGreenOff();
    ledRedOff();
}

/*!
 * @brief Reads and verifies 3 values.
 * 		  Calculates the checksum of read data and verifies it against the saved checksum.
 * 		  Turns on the green LED and sets the alaram time to read values if the read and verification was successful.
 * 		  If there was an error during read or the verification fails the LED will be set to red and alarm time will be set to 00:00:00.
 */
void readAndVerifyChecksum(void)
{
    tU8 buffer[6];
    tU16 a, b, c, calculatedChecksum;

    if (eepromPageRead(0x0000, buffer, 6) != I2C_CODE_OK)
        return FALSE;

    a = ((tU16)buffer[0] << 8) | buffer[1];
    b = ((tU16)buffer[2] << 8) | buffer[3];
    c = ((tU16)buffer[4] << 8) | buffer[5];

    calculatedChecksum = checksum3_xor(a, b, c);

    ledGreenOff();
    ledRedOff();

    if(calculatedChecksum == checksum) {
    	alarmHour = a;
    	alarmMin = b;
    	alarmSec = c;

    	ledGreenOn();
    } else {
    	alarmHour = 0;
    	alarmMin = 0;
    	alarmSec = 0;

    	ledRedOn();
    }
}

/*!
 * @brief Initializes interrupt for audio generation on timer 1. The interrupt function is executed periodically.
 *
 * @param period Period in microseconds between interrupts.
 */
static void init_irq (tU32 period)
{
    VICIntSelect &= ~TIMER_1_IRQ;
    VICVectAddr5  = (tU32)IRQ_Test;
    VICVectCntl5  = VIC_ENABLE_SLOT | TIMER_1_IRQ_NO;
    VICIntEnable  = TIMER_1_IRQ;

    T1TCR = TIMER_RESET;
    T1PR  = 0;
    T1MR0 = (((unsigned long long)period) * (unsigned long long)CORE_FREQ/1000) / 1000;
    T1IR  = TIMER_ALL_INT;
    T1MCR = MR0_I  | MR0_R;
    T1TCR = TIMER_RUN;
}

/*!
 * @brief The entry point to the program.
 * 		  Initializes operating system and creates the initialization process.
 */
int main(void)
{
	tU8 error;
	tU8 pid;

	osInit();

    osCreateProcess(initProc, initStack, INIT_STACK_SIZE, &pid, 1, NULL, &error);
	osStartProcess(pid, &error);

	osStart();
  
	return 0;
}

/*!
 * @brief Gets the hour, minute and second from the RTC.
 *
 * @param hour A pointer to hour
 * @param min A pointer to minute
 * @param sec A pointer to second
 */
void RtcGetTime(int *hour, int *min, int *sec) {
	*hour = RTC_HOUR;
	*min = RTC_MIN;
	*sec = RTC_SEC;
}

/*!
 * @brief Initializes periferals.
 * 		  Sets the P1.16-P1.19 buttons pin directions to input.
 * 		  Initializes LCD screen, sets background and text color, clears screen.
 * 		  Initializes RTC with a default time: 17:30:00.
 * 		  Initializes Digital-Auido-Converter.
 * 		  Initializes timer 0 to count milliseconds.
 * 		  Initializes ADC.
 * 		  Turns off the RGB LED.
 */
void initPeripherals(){
	IODIR1 |= (1 << 16);
	IODIR1 &= ~(1 << 20);
	IOSET1 |= (1 << 16);

	IODIR1 |= (1 << 17);
	IODIR1 &= ~(1 << 21);
	IOSET1 |= (1 << 17);

	IODIR1 |= (1 << 18);
	IODIR1 &= ~(1 << 22);
    IOSET1 |= (1 << 18);

	IODIR1 |= (1 << 19);
	IODIR1 &= ~(1 << 23);
	IOSET1 |= (1 << 19);

    lcdInit();
    lcdColor(0x00,0xff);
    lcdClrscr();
  
    RTC_CCR  = 0x00000012;
    RTC_CCR  = 0x00000010;
    RTC_ILR  = 0x00000000;
    RTC_CIIR = 0x00000000;
    RTC_AMR  = 0x00000000;

    RTC_HOUR = 17;
    RTC_MIN  = 30;
    RTC_SEC  = 0;

    RTC_CCR  = 0x00000011;

    //
	// Initialize DAC: AOUT = P0.25
	//
	PINSEL1 &= ~0x000C0000;
	PINSEL1 |=  0x00080000;

	T0TCR = 0x02;
	T0PR  = 59999;
	T0TCR = 0x01;

    initAdc();

    ledGreenOff();
    ledRedOff();
}

/*!
 * @brief Reads raw temperature value and displays it on the screen
 * 		  If the temperature reading is invalid sets the character array to "------"
 * 		  Valid temperature range is -55 <= temperature < 100
 *
 * @param tempBuf Buffer for a raw temperature value
 * @param tempStr Character array to store formatted temperature
 */
void temperture(tU8 tempBuf[], char tempStr[]) {
	i2cRead(((TEMP_ADDR) << 1) | 1, tempBuf, 2);

	tS16 rawTemp = (tempBuf[0] << 8) | tempBuf[1];
	rawTemp >>= 7;

	float temperature = rawTemp * 0.5f;

	int tempInt = (int)temperature;
	int tempFrac = (int)((temperature - tempInt) * 1000);
	if (tempFrac < 0) tempFrac = -tempFrac;
	if(temperature < -55 || temperature >= 100) {
		snprintf(tempStr, TEMP_STR_SIZE, "-----");
	}
	else {
		if (temperature < 0) {
			snprintf(tempStr, TEMP_STR_SIZE, "-%02d,%1d", -tempInt, tempFrac);
		} else {
			snprintf(tempStr, TEMP_STR_SIZE, "%02d,%1d", tempInt, tempFrac);
		}
	}

	lcdGotoxy(32 + (temperature >= 0 ? 4 : 0), 7);
	lcdPuts(tempStr);
	lcdGotoxy(32 + (temperature >= 0 ? 36 : 40), 3);
	lcdPuts(" o");
	lcdGotoxy(48 + (temperature >= 0 ? 36 : 40), 7);
	lcdPutchar('C');
}

/*!
 * @brief Check if P1.20 button is pressed and the clock is not in the alarm mode and switches the setTimeMode to the next one.
 */
void switchSetTimeMode() {
	if ((IOPIN1 & (1 << 20)) == 0 && playSound == 1) {
		if(btnsState[0] == 0) {
			setTimeMode = ++setTimeMode % 4;
			btnsState[0] = 1;
			lcdClrscr();
		}

	} else if (playSound == 1) {
		btnsState[0] = 0;
	}
}

/*!
 * @brief Check if the P1.23 button is pressed, the clock is not in the alarm mode and we are in the setTimeMode and saves the currently selected time to EPROM.
 */
void checkWriteButton() {
	if ((IOPIN1 & (1 << 23)) == 0 && playSound == 1 && setTimeMode != 0) {
		if(btnsState[3] == 0) {
			writeDataWithChecksum(alarmHourT, alarmMinT, alarmSecT);
			btnsState[3] = 1;
			lcdClrscr();
			setTimeMode = 0;
			isNewEpromValue = 1;
		}

	} else if (playSound == 1) {
		btnsState[3] = 0;
	}
}

/*!
 * @brief Displays the alarm time from setTimeMode an sets hour, minute and second value based on ADC value.
 *
 * @param timeStr A character array to save the formatted time string into.
 */
void setAlarm(char timeStr[]) {
	tU16 adcValue = getAnalogueInput(AIN1);
	tU16 x = 0;
	if(setTimeMode == 1) {
		alarmHourT = (adcValue * 23) / 1023;
		x = 32;
	} else if(setTimeMode == 2) {
		alarmMinT = (adcValue * 59) / 1023;
		x=56;
	} else if(setTimeMode == 3) {
		alarmSecT = (adcValue * 59) / 1023;
		x = 80;
	}

	snprintf(timeStr, TIME_STR_SIZE, "%02d:%02d:%02d", alarmHourT, alarmMinT, alarmSecT);
	lcdGotoxy(32,28);
	lcdPuts(timeStr);
	lcdGotoxy(x,42);
	lcdPuts("^^");
}

/*!
 * @brief Main process with the alarm functionality.
 * 		  Initializes peripherals.
 * 		  Initializes buffers for time and temperature.
 * 		  In the infinite loop:
 * 			Reads the RTC time.
 * 			Manages temperature sensor and displays the temperature.
 * 			Manages the time mode and displays time.
 * 			Manager EPROM write and read.
 * 			If the alarm time is reached it initializes sequence, audio interrupts and the DC motor.
 * 			Checks if the alarm is ready to be turned off and resets its parameters.
 *
 * @param arg A pointers to function arguments.
 */
static void proc1(void* arg)
{
	initPeripherals();

	char timeStr[TIME_STR_SIZE];
	char tempStr[TEMP_STR_SIZE];
	tU8 tempBuf[2];

	for(;;) {
		// FONT 8x14
		RtcGetTime(&hour, &minute, &sec);

		temperture(tempBuf, tempStr);
		switchSetTimeMode();
		checkWriteButton();

		if(setTimeMode == 0) {
			snprintf(timeStr, TIME_STR_SIZE, "%02d:%02d:%02d", hour, minute, sec);
			lcdGotoxy(32,28);
			lcdPuts(timeStr);
		} else {
			setAlarm(timeStr);
		}

		if(isNewEpromValue == 1) {
			readAndVerifyChecksum();
			isNewEpromValue = 0;
		}

		if(hour == alarmHour && minute == alarmMin && sec == alarmSec) {
			// Read temperature
			tU8 buf[2];
			i2cRead(((TEMP_ADDR) << 1) | 1, buf, 2);

			// Generate new sequence with temperature reading as seed
			generateRandomSequence(seq, SEQ_LEN, buf[1]);

			// Start DACR sound
			init_irq(125);
			playSound = 0;
			motorStart();
		}

		if(playSound == 0 && seqWin == 0) {
			sequenceCheck();
			ledFlashRGB();
		}

		if(seqWin == 1) {
			motorStop();
			playSound = 1;
			seqWin = 0;
			int s = 0;
			for(s = 0; s < SEQ_LEN; s++) {
				btns[s] = 0;
				seq[s] = 0;
			}
			btnsIdx = 0;
			seqCounter = 0;
			ledRedOff();
			ledGreenOff();
		}
	}
}


/*!
 *
 * @brief The entry function for the initialization process.
 *
 * @param arg - This parameter is not used in this application.
 */
static void initProc(void* arg)
{
  tU8 error;

  eaInit();   //initialize printf
  i2cInit();  //initialize I2C
  osCreateProcess(proc1, proc1Stack, PROC1_STACK_SIZE, &pid1, 3, NULL, &error);
  osStartProcess(pid1, &error);

  osDeleteProcess();
}

/*!
 *
 * @brief The timer tick entry function that is called once every timer tick
 *    	  interrupt in the RTOS. Observe that any processing in this
 *    	  function must be kept as short as possible since this function
 *    	  execute in interrupt context.
 *
 * @param elapsedTime - The number of elapsed milliseconds since last call.
 */
void
appTick(tU32 elapsedTime)
{
  msClock += elapsedTime;
}
