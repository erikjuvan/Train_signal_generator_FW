// C Standar Library
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// User Library
#include "communication.h"
#include "main.h"
#include "parse.h"
#include "uart.h"

extern void TIM_Update_ARR(uint32_t arr);
extern void TIM_Update_PSC(uint32_t psc);
extern void TIM_Update_REGS();
extern void DMA_Update(uint32_t n_entries);
extern void Stop();
extern void Start();

extern uint32_t g_pins[];
extern uint32_t g_time[];
extern uint32_t g_num_of_entries;

extern const int      IsGPIOReversePin[];
extern const uint32_t GPIOPinArray[];

extern uint8_t UART_Address;

extern CommunicationMode g_communication_mode;

static const char Delims[] = "\n\r\t, ";

static int newSettings     = 0;
static int needsCorrecting = 0;

static int timer_period_us = 0;

static void ClearSettings()
{
    for (int i = 0; i < MAX_STATES; i++) {
        g_pins[i] = g_time[i] = 0;
    }
    g_num_of_entries = 0;
}

void CorrectValues()
{
    // Shift values in the g_time array to correct for the inherent offset in the functionality
    int tmpTime = g_time[0];
    for (int i = 0; i < g_num_of_entries - 1; i++) {
        g_time[i] = g_time[i + 1];
    }
    g_time[g_num_of_entries - 1] = tmpTime;
    needsCorrecting              = 0;

    // Increase the g_time array by magic number 4 which is tied to the magic number in the PSC to get exactly 1us resolution
    for (int i = 0; i < g_num_of_entries; i++) {
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
    if (g_num_of_entries >= MAX_STATES)
        return;
    for (int i = g_num_of_entries; i > time_idx; i--) {
        g_time[i] = g_time[i - 1];
        g_pins[i] = g_pins[i - 1];
    }
    g_time[time_idx] = time_val;
    g_pins[time_idx] = 0;
    Update(ch_num, ch_idx, time_val, time_idx);
    g_num_of_entries++;
}

static void Function_VERSION(char* str, write_func Write)
{
    char buf[100] = {0};
    snprintf(buf, sizeof(buf), "VERSION,%s,%s,%s", SWVER, HWVER, COMPATIBILITYMODE);
    Write((uint8_t*)buf, strlen(buf));
}

static void Function_SETID(char* str, write_func Write)
{
    str = strtok(NULL, Delims);
    if (str != NULL) {
        int num = atoi(str);
        if (num <= 127) {
            UART_Set_Address(num);
        }
    }

    // Echo
    char buf[10] = {0};
    snprintf(buf, sizeof(buf), "SETID,%u", UART_Address);
    Write((uint8_t*)buf, strlen(buf));
}

static void Function_GETID(char* str, write_func Write)
{
    char buf[10] = {0};
    snprintf(buf, sizeof(buf), "ID,%u", UART_Address);
    Write((uint8_t*)buf, strlen(buf));
}

static void Function_PING(char* str, write_func Write)
{
    Write((uint8_t*)"PING", 4);
}

static void Function_START(char* str, write_func Write)
{
    if (needsCorrecting) {
        CorrectValues();
        DMA_Update(g_num_of_entries);
    }
    newSettings = 1;
    Start();

    // Echo
    Write((uint8_t*)"START", 5);
}

static void Function_STOP(char* str, write_func Write)
{
    newSettings = 1;
    Stop();

    // Echo
    Write((uint8_t*)"STOP", 4);
}

static void Function_SETPERIOD(char* str, write_func Write)
{
    str = strtok(NULL, Delims); // param - PERIOD [us]
    if (str != NULL) {
        int period = atoi(str);
        if (period > 0) {
            timer_period_us = period;
            TIM_Update_ARR(timer_period_us * 4); // Increase by magic number 4 which is tied to the magic number in the PSC to get exactly 1us resolution
            TIM_Update_REGS();
        }
    }

    // Echo
    char buf[30];
    snprintf(buf, sizeof(buf), "SETPERIOD,%u", timer_period_us);
    Write((uint8_t*)buf, strlen(buf));
}

// Set Channel. Example SETCH,0,140,240,32460,32560 // first param: channel number - coresponds to PIN numbers (0 - PIN0, 1 - PIN1, ...)
// second param: on g_time, third param: off g_time ... toggle so on
static void Function_SETCH(char* str, write_func Write)
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

    for (int i_el = 0; i_el < elementsFound; ++i_el) {
        if (timeArray[i_el] < g_time[0]) {
            // Lowest value doesn't exist in g_time array
            Insert(chNum, i_el, timeArray[i_el], 0);
            continue;
        } else if (timeArray[i_el] > g_time[g_num_of_entries - 1]) {
            // Highest value doesn't exist in g_time array
            Insert(chNum, i_el, timeArray[i_el], g_num_of_entries);
            continue;
        }

        // Time is somewhere in between

        found = 0;
        for (int i_ent = 0; i_ent < g_num_of_entries; i_ent++) {
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
            for (idx = 0; idx < g_num_of_entries; idx++)
                if (g_time[idx] > timeArray[i_el])
                    break;

            Insert(chNum, i_el, timeArray[i_el], idx);
        }
    }

    // Echo
    char buf[100];
    snprintf(buf, sizeof(buf), "SETCH,%u", chNum);
    for (int i = 0; i < elementsFound; ++i) {
        snprintf(&buf[strlen(buf)], sizeof(buf) - strlen(buf), ",%u", timeArray[i]);
    }
    Write((uint8_t*)buf, strlen(buf));
}

static void Function_GETPERIOD(char* str, write_func Write)
{
    char buf[30];
    snprintf(buf, sizeof(buf), "PERIOD,%u", timer_period_us);

    Write((uint8_t*)buf, strlen(buf));
}

static int WriteChannelSettings(char* buf, int max_size, int ch)
{
    int written = 0;
    buf[0]      = 0;
    // Write times for said channel
    for (int i = 0; i < g_num_of_entries; ++i) {
        if (g_pins[i] & GPIOPinArray[ch] || (g_pins[i] & GPIOPinArray[ch] << 16)) // take into account setting and reseting
        {
            if (i == 0) // fetch last time entry
                written += snprintf(&buf[strlen(buf)], max_size - strlen(buf), "%lu,", g_time[g_num_of_entries - 1] / 4);
            else
                written += snprintf(&buf[strlen(buf)], max_size - strlen(buf), "%lu,", g_time[i - 1] / 4);
        }
    }
    if (strlen(buf) > 0)
        buf[strlen(buf) - 1] = 0;

    return written;
}

static void Function_GETCH(char* str, write_func Write)
{
    char buf[100];
    int  ch = -1; // default is an invalid ch num

    // Get channel number
    /////////////////////
    str = strtok(NULL, Delims); // param - PERIOD [us]
    if (str != NULL)
        ch = atoi(str);

    if (ch < 0 || ch >= NUM_OF_CHANNELS) // Invalid channel number
        return;
    /////////////////////

    // Write channel number
    snprintf(buf, sizeof(buf), "CH,%u,", ch);

    // Write channel settings
    WriteChannelSettings(&buf[strlen(buf)], sizeof(buf) - strlen(buf), ch);

    Write((uint8_t*)buf, strlen(buf));
}

static void Function_GETSETTINGS(char* str, write_func Write)
{
    char buf[500];
    snprintf(buf, sizeof(buf), "PERIOD,%u\n", timer_period_us);

    char tmp_buf[100];
    for (int ch = 0; ch < NUM_OF_CHANNELS; ++ch) {
        if (WriteChannelSettings(tmp_buf, sizeof(tmp_buf), ch) > 0)
            snprintf(&buf[strlen(buf)], sizeof(buf) - strlen(buf), "CH,%u,%s\n", ch, tmp_buf);
    }

    Write((uint8_t*)buf, strlen(buf));
}

#define COMMAND(NAME)          \
    {                          \
#NAME, Function_##NAME \
    }

static struct {
    const char* name;
    void (*Func)(char*, write_func);
} command[] = {
    COMMAND(VERSION),
    COMMAND(SETID),
    COMMAND(GETID),
    COMMAND(PING),

    COMMAND(START),
    COMMAND(STOP),

    //COMMAND(SETFREQ), // No need for it
    COMMAND(SETPERIOD),
    COMMAND(SETCH),

    //COMMAND(GETFREQ), // No need for it
    COMMAND(GETPERIOD),
    COMMAND(GETCH),
    COMMAND(GETSETTINGS)};

/* Example program
STOP
SETPERIOD,65000
SETCH,0,140,240,32460,32560
SETCH,5,490,502
SETCH,6,752,762
SETCH,2,32810,32830
SETCH,3,33080,33100
SETCH,7,100,220,470,13970,32420,32540,32790,46290
SETCH,1,470,482,732,13982,32790,32810,33060,46560
SETCH,4,1,32560
START
*/

void Parse(char* string, write_func Write)
{
    char* str;

    str = strtok(string, Delims);
    while (str != NULL) {

        for (int i = 0; i < sizeof(command) / sizeof(command[0]); ++i) {
            if (strcmp(str, command[i].name) == 0) {
                command[i].Func(str, Write);
                break;
            }
        }

        str = strtok(NULL, Delims);
    }
}