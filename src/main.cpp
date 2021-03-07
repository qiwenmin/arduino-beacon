#include "Arduino.h"
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
#include "RTClib.h"
#include "OneWire.h"
#include "DallasTemperature.h"
#include "si5351.h"
#include "JTEncode.h"

// GPIO
#define SW_BTN 5
#define DS 6
#define TX 7

// SSD1306
#define SSD1306_ADDRESS 0x3C

SSD1306AsciiAvrI2c oled;

// DS1307 with DS18B20
RTC_DS1307 rtc;
OneWire oneWire(DS);
DallasTemperature sensors(&oneWire);

// Si5351
#define F_XTAL 25002305
Si5351 si5351;

// WSPR
const uint64_t wspr_tone_space = 146 * SI5351_FREQ_MULT / 100;

#define WSPR_CTC 10672

JTEncode jtencode;
const uint32_t wspr_freqs[] = {
    5288700,
    7040100,
};
const int wspr_freqs_count = sizeof wspr_freqs / sizeof wspr_freqs[0];

char callsign[] = "BG1REN";
char loc[] = "ON80";
uint8_t dbm = 27; // 500mW
uint8_t wspr_tx_buffer[WSPR_SYMBOL_COUNT];

volatile bool proceed = false;

ISR(TIMER1_COMPA_vect) {
    proceed = true;
}

static void set5351freq(uint64_t freq, si5351_clock clock) {
    si5351.set_freq_manual(freq * SI5351_FREQ_MULT, SI5351_PLL_FIXED, clock);
}

static void printError(const __FlashStringHelper *msg) {
    Serial.println(msg);
    oled.setCursor(0, 6);
    oled.print(msg);
}

static void printDebug(const __FlashStringHelper *msg) {
    Serial.println(msg);
}

void setup() {
    // GPIO
    digitalWrite(TX, LOW);
    pinMode(TX, OUTPUT);

    pinMode(SW_BTN, INPUT_PULLUP);

    // Serial
    while (!Serial);
    Serial.begin(38400);

    // OLED
    oled.begin(&Adafruit128x64, SSD1306_ADDRESS);
    oled.setFont(ZevvPeep8x16);
    oled.clear();

    // DS1307
    if (!rtc.begin()) {
        printError(F("No RTC!"));
    } else if (!rtc.isrunning()) {
        printDebug(F("RTC is not running. Initial the datetime."));
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }

    // DS18B20
    sensors.setWaitForConversion(false); // non-blocking

    // Si5351
    si5351.init(SI5351_CRYSTAL_LOAD_8PF, F_XTAL, 0);
    si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
    si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA);
    si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA);
    si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_2MA);

    si5351.set_clock_disable(SI5351_CLK1, SI5351_CLK_DISABLE_LOW);
    si5351.output_enable(SI5351_CLK1, 0);
    si5351.set_clock_disable(SI5351_CLK2, SI5351_CLK_DISABLE_LOW);
    si5351.output_enable(SI5351_CLK2, 0);

    set5351freq(0, SI5351_CLK0);

    // wspr
    noInterrupts();
    TCCR1A = 0;
    TCNT1 = 0;
    TCCR1B = (1 << CS12) |
        (1 << CS10) |
        (1 << WGM12);
    TIMSK1 = (1 << OCIE1A);
    OCR1A = WSPR_CTC;
    interrupts();

    jtencode.wspr_encode(callsign, loc, dbm, wspr_tx_buffer);
}

static char buf32[32];

static uint32_t last_unixtime = 0;
static char daysOfTheWeek[7][4] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
static unsigned long last_temperature_request_at = 0;

static int wspr_p = -1;
static bool wspr_waiting_proceed = false;
static int wspr_freq_idx = 0;
static uint64_t wspr_freq = 0;

void loop() {
    auto current_millis = millis();

    auto now = rtc.now();
    auto current_unixtime = now.unixtime();

    if (wspr_p >= 0) {
        digitalWrite(TX, HIGH);

        // process wspr
        if (wspr_waiting_proceed) {
            if (proceed) {
                wspr_p ++;
                if (wspr_p >= WSPR_SYMBOL_COUNT) {
                    wspr_p = -1;
                    digitalWrite(TX, LOW);
                    printDebug(F("WSPR Done."));
                    oled.setCursor(0, 0);
                    oled.print(F("              "));
                }

                wspr_waiting_proceed = false;
            }
        } else {
            si5351.set_freq(wspr_freq + wspr_tx_buffer[wspr_p] * wspr_tone_space, SI5351_CLK0);
            proceed = false;
            wspr_waiting_proceed = true;
        }
    }

    if (current_unixtime != last_unixtime) { // every new second
        last_unixtime = current_unixtime;

        // sending wspr?
        if (wspr_p == -1 && now.second() == 0 && (now.minute() & 0x01) == 0) { // sending wspr every 2 minutes
            wspr_p = 0;
            auto wspr_freq_hz = wspr_freqs[wspr_freq_idx] + (now.minute() >> 1);
            wspr_freq = wspr_freq_hz * SI5351_FREQ_MULT;
            wspr_freq_idx ++;
            if (wspr_freq_idx >= wspr_freqs_count) {
                wspr_freq_idx = 0;
            }

            printDebug(F("WSPR TXing..."));

            // display wspr freq:
            sprintf(buf32, "WSPR: %8ld", wspr_freq_hz);
            oled.setCursor(0, 0);
            oled.print(buf32);
        }

        // update display
        sprintf(buf32, "%04d/%02d/%02d %s", now.year(), now.month(), now.day(), daysOfTheWeek[now.dayOfTheWeek()]);
        oled.setCursor(8, 4);
        oled.print(buf32);

        sprintf(buf32, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
        oled.setCursor(8, 6);
        oled.print(buf32);

        if ((current_millis - last_temperature_request_at) > 750 && last_temperature_request_at != 0) {
            auto temperature = sensors.getTempCByIndex(0);
            char t_str[8];
            dtostrf(temperature, 4, 1, t_str);
            sprintf(buf32, "%sC", t_str);
            oled.setCursor(80, 6);
            oled.print(buf32);
        }

        last_temperature_request_at = current_millis;
        sensors.requestTemperatures();
    }
}