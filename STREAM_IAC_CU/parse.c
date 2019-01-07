// C Standar Library
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// User Library
#include "communication.h"
#include "main.h"
#include "uart.h"

extern void TIM_Update_ARR(uint32_t arr);
extern void TIM_Update_PSC(uint32_t psc);
extern void TIM_Update_REGS();
extern void DMA_Update(uint32_t n_entries);
extern void Stop();
extern void Start();

extern uint32_t g_pins[];
extern uint32_t g_time[];
extern uint32_t num_of_entries;

extern const int      IsGPIOReversePin[];
extern const uint32_t GPIOPinArray[];

extern uint8_t UART_Address;

extern CommunicationMode      g_communication_mode;
extern CommunicationInterface g_communication_interface;

const char Delims[] = "\n\r\t, ";

int newSettings     = 0;
int needsCorrecting = 0;

int timerARR = 0;
int timerPSC = 0;

static void ClearSettings()
{
    for (int i = 0; i < MAX_STATES; i++) {
        g_pins[i] = g_time[i] = 0;
    }
    num_of_entries = 0;
}

void CorrectValues()
{
    // Shift values in the g_time array to correct for the inherent offset in the functionality
    int tmpTime = g_time[0];
    for (int i = 0; i < num_of_entries - 1; i++) {
        g_time[i] = g_time[i + 1];
    }
    g_time[num_of_entries - 1] = tmpTime;
    needsCorrecting            = 0;

    // Increase the g_time array by magic number 4 which is tied to the magic number in the PSC to get exactly 1us resolution
    for (int i = 0; i < num_of_entries; i++) {
        g_time[i] *= 4;
    }
}

static int StrToInts(char* str, int* ints, int maxArrSize)
{
    int  element    = 0;
    int  num_i      = 0;
    char number[10] = {0};
    char ch;

    for (int i = 0; str[i] != '\0'; i++) {
        ch = str[i];
        if (ch >= '0' && ch <= '9') {
            number[num_i++] = ch;
        } else if (ch == ',') {
            if (num_i > 0) {
                if (element >= maxArrSize)
                    break;
                ints[element++] = atoi(number);

                num_i = 0;

                for (int i = 0; i < sizeof(number); i++)
                    number[i] = 0;
            }
        }
    }

    if (num_i > 0 && element < maxArrSize) {
        ints[element++] = atoi(number);
    }

    return element;
}

static void Update(uint32_t ch_num, int ch_idx, int time_val, int time_idx)
{
    uint32_t gpio_pin = GPIOPinArray[ch_num];

    if (!IsGPIOReversePin[ch_num]) {
        // Pin is NOT reversed
        g_pins[time_idx] |= ch_idx % 2 == 0 ? gpio_pin : gpio_pin << 16;
    } else {
        // Pin is reversed
        g_pins[time_idx] |= ch_idx % 2 == 1 ? gpio_pin : gpio_pin << 16;
    }
}

static void Insert(uint32_t ch_num, int ch_idx, int time_val, int time_idx)
{
    if (num_of_entries >= MAX_STATES)
        return;
    for (int i = num_of_entries; i > time_idx; i--) {
        g_time[i] = g_time[i - 1];
        g_pins[i] = g_pins[i - 1];
    }
    g_time[time_idx] = time_val;
    g_pins[time_idx] = 0;
    Update(ch_num, ch_idx, time_val, time_idx);
    num_of_entries++;
}

static void Function_Unknown(char* str)
{
    __asm__("nop");
}

// Enable/Disable. Example: CENBL,0 - disable output, CENBL,1 - enable output
static void Function_CENBL(char* str)
{
    str = strtok(NULL, Delims); // first parameter

    if (str[0] == '0') { // DISABLE
        newSettings = 1;
        Stop();
    } else if (str[0] == '1') { // ENABLE
        if (needsCorrecting) {
            CorrectValues();
            DMA_Update(num_of_entries);
        }
        newSettings = 1;
        Start();
    }
}

// Set settings. Example: CPRDS,1000000,65000 // first param: Timer base frequency [Hz], second param: Timer Period [us]
static void Function_CPRDS(char* str)
{
    str = strtok(NULL, Delims); // first param - FREQUENCY(RESOLUTION) !NOT IMPLEMENTED!
    /* if(str != NULL) {   ALL THIS IS PREVIOUS CODE, NOT IMPLEMENTED FOR NEW SCRIPT FORMAT
		int num = atoi(str);
		if (num > 0) {
			timerPSC = num;
			TIM_Update_PSC(timerPSC);
			TIM_Update_REGS();
		}
	} */

    str = strtok(NULL, Delims); // second param	- PERIOD
    if (str != NULL) {
        int num = atoi(str);
        if (num > 0) {
            timerARR = num;
            TIM_Update_ARR(timerARR * 4); // Increase by magic number 4 which is tied to the magic number in the PSC to get exactly 1us resolution
            TIM_Update_REGS();
        }
    }
}

// Set Channel. Example CCHNL,0,140,240,32460,32560 // first param: channel number - coresponds to PIN numbers (0 - PIN0, 1 - PIN1, ...)
// second param: on g_time, third param: off g_time ... toggle so on
static void Function_CCHNL(char* str)
{
    int found = 0;

    if (newSettings) {
        needsCorrecting = 1;
        ClearSettings();
        newSettings = 0;
    }

    str = strtok(NULL, Delims);
    if (str == NULL)
        return;
    unsigned int chNum = atoi(str);
    if (chNum >= NUM_OF_CHANNELS)
        return;

    str = strtok(NULL, "\n\r");

    int timeArray[20] = {0};
    int elementsFound = StrToInts(str, timeArray, sizeof(timeArray) / sizeof(*timeArray));

    for (int i_el = 0; i_el < elementsFound; i_el++) {
        if (timeArray[i_el] < g_time[0]) {
            // Lowest value doesn't exist in g_time array
            Insert(chNum, i_el, timeArray[i_el], 0);
            continue;
        } else if (timeArray[i_el] > g_time[num_of_entries - 1]) {
            // Highest value doesn't exist in g_time array
            Insert(chNum, i_el, timeArray[i_el], num_of_entries);
            continue;
        }

        // Time is somewhere in between

        found = 0;
        for (int i_ent = 0; i_ent < num_of_entries; i_ent++) {
            if (g_time[i_ent] == timeArray[i_el]) {
                // Time already exists
                Update(chNum, i_el, g_time[i_ent], i_ent);
                found = 1;
                break;
            }
        }

        // Time does not exist, insert it into array
        if (!found) {
            int idx = 0;
            for (idx = 0; idx < num_of_entries; idx++) {
                if (g_time[idx] > timeArray[i_el])
                    break;
            }
            Insert(chNum, i_el, timeArray[i_el], idx);
        }
    }
}

static void Function_STRT(char* str)
{
}

static void Function_VERG(char* str)
{
    char buf[100] = {0};
    strncpy(&buf[strlen(buf)], "VERG,", 5);
    strncpy(&buf[strlen(buf)], SWVER, strlen(SWVER));
    buf[strlen(buf)] = ',';
    strncpy(&buf[strlen(buf)], HWVER, strlen(HWVER));
    buf[strlen(buf)] = ',';
    strncpy(&buf[strlen(buf)], COMPATIBILITYMODE, strlen(COMPATIBILITYMODE));
    Write((uint8_t*)buf, strlen(buf));
}

static void Function_IDST(char* str)
{
    str = strtok(NULL, Delims);
    if (str != NULL) {
        int num = atoi(str);
        if (num <= 127) {
            UART_Set_Address(num);
        }
    }
}

static void Function_IDGT(char* str)
{
    char buf[10] = {0};
    strncpy(buf, "ID:", 3);
    itoa(UART_Address, &buf[strlen(buf)], 10);

    Write((uint8_t*)buf, strlen(buf));
}

static void Function_USBY(char* str)
{
    g_communication_interface = USB;
}

static void Function_USBN(char* str)
{
    g_communication_interface = UART;
}

static void Function_PING(char* str)
{
    Write((uint8_t*)"OK", 2);
}

#define COMMAND(NAME)          \
    {                          \
#NAME, Function_##NAME \
    }

static struct {
    const char* name;
    void (*Func)(char*);
} command[] = {
    COMMAND(STRT),
    COMMAND(VERG),
    COMMAND(IDST),
    COMMAND(IDGT),
    COMMAND(USBY),
    COMMAND(USBN),
    COMMAND(PING),

    // Legacy
    COMMAND(CENBL),
    COMMAND(CPRDS),
    COMMAND(CCHNL)};

/* Example program
CENBL,0
CPRDS,1000000,65000		
CCHNL,0,140,240,32460,32560
CCHNL,5,490,502
CCHNL,6,752,762
CCHNL,2,32810,32830
CCHNL,3,33080,33100
CCHNL,7,100,220,470,13970,32420,32540,32790,46290
CCHNL,1,470,482,732,13982,32790,32810,33060,46560
CCHNL,4,1,32560
CENBL,1
*/

void Parse(char* string)
{
    char* str;

    str = strtok(string, Delims);
    while (str != NULL) {

        for (int i = 0; i < sizeof(command) / sizeof(command[0]); ++i) {
            if (strcmp(str, command[i].name) == 0) {
                command[i].Func(str);
                break;
            }
        }

        str = strtok(NULL, Delims);
    }
}