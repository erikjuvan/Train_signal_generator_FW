// C Standard Library
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// User Library
#include "communication.h"
#include "main.h"
#include "parse.h"
#include "uart.h"

extern void StopRequest();
extern void StartRequest();

extern uint32_t g_pins_shadow[MAX_STATES];
extern uint32_t g_time_shadow[MAX_STATES];
extern uint32_t g_num_of_entries;

extern char g_new_settings_received;

extern const int      IsGPIOReversePin[];
extern const uint32_t GPIOPinArray[];

extern uint8_t UART_Address;

static const char Delims[] = "\n\r\t, ";

static int newSettings     = 0;
static int needsCorrecting = 0;

extern int g_timer_period_us;

static void ClearSettings()
{
    for (int i = 0; i < MAX_STATES; i++) {
        g_pins_shadow[i] = g_time_shadow[i] = 0;
    }
    g_num_of_entries = 0;
}

void CorrectValues()
{
    // Shift values in the g_time array to correct for the inherent offset in the functionality
    int tmpTime = g_time_shadow[0];
    for (int i = 0; i < g_num_of_entries - 1; i++) {
        g_time_shadow[i] = g_time_shadow[i + 1];
    }
    g_time_shadow[g_num_of_entries - 1] = tmpTime;
    needsCorrecting                     = 0;
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
        g_pins_shadow[time_idx] |= ch_idx % 2 == 0 ? gpio_pin : gpio_pin << 16;
    } else {
        // Pin is reversed
        g_pins_shadow[time_idx] |= ch_idx % 2 == 1 ? gpio_pin : gpio_pin << 16;
    }
}

static void Insert(uint32_t ch_num, int ch_idx, int time_val, int time_idx)
{
    if (g_num_of_entries >= MAX_STATES)
        return;
    for (int i = g_num_of_entries; i > time_idx; i--) {
        g_time_shadow[i] = g_time_shadow[i - 1];
        g_pins_shadow[i] = g_pins_shadow[i - 1];
    }
    g_time_shadow[time_idx] = time_val;
    g_pins_shadow[time_idx] = 0;
    Update(ch_num, ch_idx, time_val, time_idx);
    g_num_of_entries++;
}

static void Function_RSET(char* str, write_func Write)
{
    // Echo
    Write((uint8_t*)"RSET", 4);

    // Give it time to send string back
    HAL_Delay(100);

    NVIC_SystemReset();
}

static void Function_VERG(char* str, write_func Write)
{
    char buf[100] = {0};
    snprintf(buf, sizeof(buf), "VERG,%s,%s,%s", SWVER, HWVER, COMPATIBILITYMODE);
    Write((uint8_t*)buf, strlen(buf));
}

static void Function_ID_S(char* str, write_func Write)
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
    snprintf(buf, sizeof(buf), "ID_S,%u", UART_Address);
    Write((uint8_t*)buf, strlen(buf));
}

static void Function_ID_G(char* str, write_func Write)
{
    char buf[10] = {0};
    snprintf(buf, sizeof(buf), "ID_G,%u", UART_Address);
    Write((uint8_t*)buf, strlen(buf));
}

static void Function_PING(char* str, write_func Write)
{
    Write((uint8_t*)"PING", 4);
}

static void Function_STRT(char* str, write_func Write)
{
    if (needsCorrecting) {
        CorrectValues();
        g_new_settings_received = 1;
    }
    newSettings = 1;
    StartRequest();

    // Echo
    Write((uint8_t*)"STRT", 4);
}

static void Function_STOP(char* str, write_func Write)
{
    newSettings = 1;
    StopRequest();

    // Echo
    Write((uint8_t*)"STOP", 4);
}

static void Function_PRDS(char* str, write_func Write)
{
    str = strtok(NULL, Delims); // param - PERIOD [us]
    if (str != NULL) {
        int period = atoi(str);
        if (period > 0) {
            // This also triggers new settings received
            g_new_settings_received = 1;
            g_timer_period_us       = period;
        }
    }

    // Echo
    char buf[30];
    snprintf(buf, sizeof(buf), "PRDS,%u", g_timer_period_us);
    Write((uint8_t*)buf, strlen(buf));
}

// Set Channel. Example CHLS,0,140,240,32460,32560 // first param: channel number - coresponds to PIN numbers (0 - PIN0, 1 - PIN1, ...)
// second param: on g_time, third param: off g_time ... toggle so on
static void Function_CHLS(char* str, write_func Write)
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
        if (timeArray[i_el] < g_time_shadow[0]) {
            // Lowest value doesn't exist in g_time array
            Insert(chNum, i_el, timeArray[i_el], 0);
            continue;
        } else if (timeArray[i_el] > g_time_shadow[g_num_of_entries - 1]) {
            // Highest value doesn't exist in g_time array
            Insert(chNum, i_el, timeArray[i_el], g_num_of_entries);
            continue;
        }

        // Time is somewhere in between

        found = 0;
        for (int i_ent = 0; i_ent < g_num_of_entries; i_ent++) {
            if (g_time_shadow[i_ent] == timeArray[i_el]) {
                // Time already exists
                Update(chNum, i_el, g_time_shadow[i_ent], i_ent);
                found = 1;
                break;
            }
        }

        // Time does not exist, insert it into array
        if (!found) {
            int idx = 0;
            for (idx = 0; idx < g_num_of_entries; idx++)
                if (g_time_shadow[idx] > timeArray[i_el])
                    break;

            Insert(chNum, i_el, timeArray[i_el], idx);
        }
    }

    // Echo
    char buf[100];
    snprintf(buf, sizeof(buf), "CHLS,%u", chNum);
    for (int i = 0; i < elementsFound; ++i) {
        snprintf(&buf[strlen(buf)], sizeof(buf) - strlen(buf), ",%u", timeArray[i]);
    }
    Write((uint8_t*)buf, strlen(buf));
}

static void Function_PRDG(char* str, write_func Write)
{
    char buf[30];
    snprintf(buf, sizeof(buf), "PRDG,%u", g_timer_period_us);

    Write((uint8_t*)buf, strlen(buf));
}

static int WriteChannelSettings(char* buf, int max_size, int ch)
{
    int written = 0;
    buf[0]      = 0;
    // Write times for said channel
    for (int i = 0; i < g_num_of_entries; ++i) {
        if (g_pins_shadow[i] & GPIOPinArray[ch] || (g_pins_shadow[i] & GPIOPinArray[ch] << 16)) // take into account setting and reseting
        {
            if (i == 0) // fetch last time entry
                written += snprintf(&buf[strlen(buf)], max_size - strlen(buf), "%lu,", g_time_shadow[g_num_of_entries - 1]);
            else
                written += snprintf(&buf[strlen(buf)], max_size - strlen(buf), "%lu,", g_time_shadow[i - 1]);
        }
    }
    if (strlen(buf) > 0)
        buf[strlen(buf) - 1] = 0;

    return written;
}

static void Function_CHLG(char* str, write_func Write)
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
    snprintf(buf, sizeof(buf), "CHLG,%u,", ch);

    // Write channel settings
    WriteChannelSettings(&buf[strlen(buf)], sizeof(buf) - strlen(buf), ch);

    Write((uint8_t*)buf, strlen(buf));
}

static void Function_STTG(char* str, write_func Write)
{
    char buf[500];
    snprintf(buf, sizeof(buf), "PERIOD,%u\n", g_timer_period_us);

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
    COMMAND(VERG), // GET VERION
    COMMAND(ID_S), // SET ID
    COMMAND(ID_G), // GET ID
    COMMAND(PING), // PING (echo)
    COMMAND(RSET), // RESET

    COMMAND(STRT), // START
    COMMAND(STOP), // STOP

    COMMAND(PRDS), // SET PERIOD
    COMMAND(CHLS), // SET CHANNEL

    COMMAND(PRDG), // GET PERIOD
    COMMAND(CHLG), // GET CHANNEL
    COMMAND(STTG), // GET ALL SETTINGS
};

/* Example program
STOP
PRDS,65000
CHLS,0,140,240,32460,32560
CHLS,5,490,502
CHLS,6,752,762
CHLS,2,32810,32830
CHLS,3,33080,33100
CHLS,7,100,220,470,13970,32420,32540,32790,46290
CHLS,1,470,482,732,13982,32790,32810,33060,46560
CHLS,4,1,32560
STRT
*/

void Parse(char* string, write_func Write)
{
    char* str;

    str = strtok(string, Delims);
    while (str != NULL) {

        for (int i = 0; i < sizeof(command) / sizeof(command[0]); ++i) {
            if (*(uint32_t*)str == *(uint32_t*)command[i].name) {
                command[i].Func(str, Write);
                break;
            }
        }

        str = strtok(NULL, Delims);
    }
}