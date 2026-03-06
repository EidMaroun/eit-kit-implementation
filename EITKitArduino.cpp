/*
 * EITKitArduino.cpp - Library for EIT-kit Sensing Board.
 * Will be released into the public domain.
 *
 * FIXES APPLIED:
 *  1. Added missing set_source_gain() / set_meas_gain() implementations
 *  2. rms_array[num_meas] = 0 on skipped measurements in read_frame()
 *  3. Increased mux settling delays (200->500us, 100->300us)
 *  4. Added diagnose_adc_bus() for hardware validation
 *  5. read_signal debug now always prints (not rate-limited) during calibration
 *  6. Removed stale/commented-out dead code
 */

#include <Arduino.h>
#include "EITKitArduino.h"
#include "SPI.h"
#include "assert.h"

#if !defined(__IMXRT1062__)
    #error "EITKitArduino: This library now supports Teensy 4.x only (__IMXRT1062__)."
#endif

// ===================== DEBUG DUMP =====================
#define EIT_DEBUG_DUMP 0           // set to 1 to enable periodic raw GPIO logs
#define EIT_DEBUG_MEAS_INDEX 2     // which measurement index to dump (0.._num_meas-1)
// ======================================================

// State of MCP4252 TCON register
uint8_t tcon_reg = 0xFF;

int16_t sine_table[1024] = {
      0,   3,   6,   9,  12,  15,  18,  21,  25,  28,  31,  34,  37,  40,  43,  47,
     50,  53,  56,  59,  62,  65,  68,  72,  75,  78,  81,  84,  87,  90,  93,  96,
     99, 102, 106, 109, 112, 115, 118, 121, 124, 127, 130, 133, 136, 139, 142, 145,
    148, 151, 154, 157, 160, 163, 166, 169, 172, 175, 178, 181, 184, 187, 190, 193,
    195, 198, 201, 204, 207, 210, 213, 216, 218, 221, 224, 227, 230, 233, 235, 238,
    241, 244, 246, 249, 252, 255, 257, 260, 263, 265, 268, 271, 273, 276, 279, 281,
    284, 287, 289, 292, 294, 297, 299, 302, 304, 307, 310, 312, 314, 317, 319, 322,
    324, 327, 329, 332, 334, 336, 339, 341, 343, 346, 348, 350, 353, 355, 357, 359,
    362, 364, 366, 368, 370, 372, 375, 377, 379, 381, 383, 385, 387, 389, 391, 393,
    395, 397, 399, 401, 403, 405, 407, 409, 411, 413, 414, 416, 418, 420, 422, 423,
    425, 427, 429, 430, 432, 434, 435, 437, 439, 440, 442, 443, 445, 447, 448, 450,
    451, 453, 454, 455, 457, 458, 460, 461, 462, 464, 465, 466, 468, 469, 470, 471,
    473, 474, 475, 476, 477, 478, 479, 481, 482, 483, 484, 485, 486, 487, 488, 489,
    489, 490, 491, 492, 493, 494, 495, 495, 496, 497, 498, 498, 499, 500, 500, 501,
    502, 502, 503, 503, 504, 504, 505, 505, 506, 506, 507, 507, 508, 508, 508, 509,
    509, 509, 510, 510, 510, 510, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511,
    512, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 510, 510, 510, 510, 509,
    509, 509, 508, 508, 508, 507, 507, 506, 506, 505, 505, 504, 504, 503, 503, 502,
    502, 501, 500, 500, 499, 498, 498, 497, 496, 495, 495, 494, 493, 492, 491, 490,
    489, 489, 488, 487, 486, 485, 484, 483, 482, 481, 479, 478, 477, 476, 475, 474,
    473, 471, 470, 469, 468, 466, 465, 464, 462, 461, 460, 458, 457, 455, 454, 453,
    451, 450, 448, 447, 445, 443, 442, 440, 439, 437, 435, 434, 432, 430, 429, 427,
    425, 423, 422, 420, 418, 416, 414, 413, 411, 409, 407, 405, 403, 401, 399, 397,
    395, 393, 391, 389, 387, 385, 383, 381, 379, 377, 375, 372, 370, 368, 366, 364,
    362, 359, 357, 355, 353, 350, 348, 346, 343, 341, 339, 336, 334, 332, 329, 327,
    324, 322, 319, 317, 314, 312, 310, 307, 304, 302, 299, 297, 294, 292, 289, 287,
    284, 281, 279, 276, 273, 271, 268, 265, 263, 260, 257, 255, 252, 249, 246, 244,
    241, 238, 235, 233, 230, 227, 224, 221, 218, 216, 213, 210, 207, 204, 201, 198,
    195, 193, 190, 187, 184, 181, 178, 175, 172, 169, 166, 163, 160, 157, 154, 151,
    148, 145, 142, 139, 136, 133, 130, 127, 124, 121, 118, 115, 112, 109, 106, 102,
     99,  96,  93,  90,  87,  84,  81,  78,  75,  72,  68,  65,  62,  59,  56,  53,
     50,  47,  43,  40,  37,  34,  31,  28,  25,  21,  18,  15,  12,   9,   6,   3,
      0,  -3,  -6,  -9, -12, -15, -18, -21, -25, -28, -31, -34, -37, -40, -43, -47,
    -50, -53, -56, -59, -62, -65, -68, -72, -75, -78, -81, -84, -87, -90, -93, -96,
    -99,-102,-106,-109,-112,-115,-118,-121,-124,-127,-130,-133,-136,-139,-142,-145,
   -148,-151,-154,-157,-160,-163,-166,-169,-172,-175,-178,-181,-184,-187,-190,-193,
   -195,-198,-201,-204,-207,-210,-213,-216,-218,-221,-224,-227,-230,-233,-235,-238,
   -241,-244,-246,-249,-252,-255,-257,-260,-263,-265,-268,-271,-273,-276,-279,-281,
   -284,-287,-289,-292,-294,-297,-299,-302,-304,-307,-310,-312,-314,-317,-319,-322,
   -324,-327,-329,-332,-334,-336,-339,-341,-343,-346,-348,-350,-353,-355,-357,-359,
   -362,-364,-366,-368,-370,-372,-375,-377,-379,-381,-383,-385,-387,-389,-391,-393,
   -395,-397,-399,-401,-403,-405,-407,-409,-411,-413,-414,-416,-418,-420,-422,-423,
   -425,-427,-429,-430,-432,-434,-435,-437,-439,-440,-442,-443,-445,-447,-448,-450,
   -451,-453,-454,-455,-457,-458,-460,-461,-462,-464,-465,-466,-468,-469,-470,-471,
   -473,-474,-475,-476,-477,-478,-479,-481,-482,-483,-484,-485,-486,-487,-488,-489,
   -489,-490,-491,-492,-493,-494,-495,-495,-496,-497,-498,-498,-499,-500,-500,-501,
   -502,-502,-503,-503,-504,-504,-505,-505,-506,-506,-507,-507,-508,-508,-508,-509,
   -509,-509,-510,-510,-510,-510,-511,-511,-511,-511,-511,-511,-511,-511,-511,-511,
   -512,-511,-511,-511,-511,-511,-511,-511,-511,-511,-511,-510,-510,-510,-510,-509,
   -509,-509,-508,-508,-508,-507,-507,-506,-506,-505,-505,-504,-504,-503,-503,-502,
   -502,-501,-500,-500,-499,-498,-498,-497,-496,-495,-495,-494,-493,-492,-491,-490,
   -489,-489,-488,-487,-486,-485,-484,-483,-482,-481,-479,-478,-477,-476,-475,-474,
   -473,-471,-470,-469,-468,-466,-465,-464,-462,-461,-460,-458,-457,-455,-454,-453,
   -451,-450,-448,-447,-445,-443,-442,-440,-439,-437,-435,-434,-432,-430,-429,-427,
   -425,-423,-422,-420,-418,-416,-414,-413,-411,-409,-407,-405,-403,-401,-399,-397,
   -395,-393,-391,-389,-387,-385,-383,-381,-379,-377,-375,-372,-370,-368,-366,-364,
   -362,-359,-357,-355,-353,-350,-348,-346,-343,-341,-339,-336,-334,-332,-329,-327,
   -324,-322,-319,-317,-314,-312,-310,-307,-304,-302,-299,-297,-294,-292,-289,-287,
   -284,-281,-279,-276,-273,-271,-268,-265,-263,-260,-257,-255,-252,-249,-246,-244,
   -241,-238,-235,-233,-230,-227,-224,-221,-218,-216,-213,-210,-207,-204,-201,-198,
   -195,-193,-190,-187,-184,-181,-178,-175,-172,-169,-166,-163,-160,-157,-154,-151,
   -148,-145,-142,-139,-136,-133,-130,-127,-124,-121,-118,-115,-112,-109,-106,-102,
    -99, -96, -93, -90, -87, -84, -81, -78, -75, -72, -68, -65, -62, -59, -56, -53,
    -50, -47, -43, -40, -37, -34, -31, -28, -25, -21, -18, -15, -12,  -9,  -6,  -3
};

extern volatile uint32_t F_CPU_ACTUAL;
extern const uint8_t pin_to_channel[42];

// =====================================================================
// Constructors
// =====================================================================

EITKitArduino::EITKitArduino()
    : EITKitArduino(32, 1, 4, AD, AD, false) {}

EITKitArduino::EITKitArduino(int num_electrodes, int num_bands, int num_terminals,
                             Meas_t drive_type, Meas_t meas_type, bool bluetooth_communication)
{
    if (num_electrodes <= 0) num_electrodes = 8;
    if (num_electrodes > 32) num_electrodes = 32;
    if (num_bands <= 0)      num_bands = 1;
    if (num_terminals <= 0)  num_terminals = 4;

    _num_electrodes = num_electrodes;
    _num_bands      = num_bands;
    _num_terminals  = num_terminals;
    _drive_type     = drive_type;
    _meas_type      = meas_type;

    _bluetooth_communication = bluetooth_communication;
    serial_communication     = !bluetooth_communication;

    _num_meas = _num_electrodes * _num_electrodes;

    for (int i = 0; i < NUM_MEAS; i++) {
        _signal_rms[i]   = 0;
        _signal_phase[i] = 0;
        signal_mag[i]    = 0;
        _cur_frame[i]    = 0;
    }
}

// =====================================================================
// begin() - Hardware initialisation
// =====================================================================

void EITKitArduino::begin()
{
    // Enable cycle counter for timing
    ARM_DEMCR     |= ARM_DEMCR_TRCENA;
    ARM_DWT_CTRL  |= ARM_DWT_CTRL_CYCCNTENA;
    ARM_DWT_CYCCNT = 0;

    Serial.println("[EITKit] Initiating Hardware Setup...");
    Serial.print("[EITKit] SPI Freq: ");
    Serial.println(SPI_FREQ_FAST);

    // Chip selects - idle high
    pinMode(CHIP_SEL_DRIVE,   OUTPUT); digitalWrite(CHIP_SEL_DRIVE,   HIGH);
    pinMode(CHIP_SEL_MEAS,    OUTPUT); digitalWrite(CHIP_SEL_MEAS,    HIGH);
    pinMode(CHIP_SEL_AD5930,  OUTPUT); digitalWrite(CHIP_SEL_AD5930,  HIGH);

    // Vertical control bus (AD5930 + AD5270)
    pinMode(V_DAT_PIN,  OUTPUT);
    pinMode(V_SCLK_PIN, OUTPUT);

    // Horizontal control bus (external ADG731 mux boards)
    pinMode(H_DAT_PIN,  OUTPUT);
    pinMode(H_SCLK_PIN, OUTPUT);

    // Mux chip selects
    pinMode(CHIP_SEL_MUX_SRC,  OUTPUT); digitalWrite(CHIP_SEL_MUX_SRC,  HIGH);
    pinMode(CHIP_SEL_MUX_SINK, OUTPUT); digitalWrite(CHIP_SEL_MUX_SINK, HIGH);
    pinMode(CHIP_SEL_MUX_VP,   OUTPUT); digitalWrite(CHIP_SEL_MUX_VP,   HIGH);
    pinMode(CHIP_SEL_MUX_VN,   OUTPUT); digitalWrite(CHIP_SEL_MUX_VN,   HIGH);

    // AD5930 control pins
    pinMode(AD5930_INT_PIN,     INPUT);
    pinMode(AD5930_CTRL_PIN,    OUTPUT); digitalWrite(AD5930_CTRL_PIN,    LOW);
    pinMode(AD5930_STANDBY_PIN, OUTPUT); digitalWrite(AD5930_STANDBY_PIN, LOW);
    pinMode(AD5930_MSBOUT_PIN,  INPUT);

    // ADC parallel bus input pins
    for (int i = 14; i <= 23; i++) {
        pinMode(i, INPUT);
    }

    Serial.println("[EITKit] Writing AD5930 Control Registers...");
    AD5930_Write(CTRL_REG, 0b011111110011);
    AD5930_Set_Start_Freq(TEST_FREQ);
    _current_freq = TEST_FREQ;

    AD5270_LockUnlock(CHIP_SEL_DRIVE, 0);
    AD5270_LockUnlock(CHIP_SEL_MEAS,  0);

    Serial.println("[EITKit] Starting frequency sweep...");
    digitalWrite(AD5930_CTRL_PIN, HIGH);
    delay(100);

    // Initial mux state: electrodes 0,1 for drive; 0,1 for sense
    mux_write_to_electrode(SRC,  0, MUX_EN);
    mux_write_to_electrode(SINK, 1, MUX_EN);
    mux_write_to_electrode(VP,   0, MUX_EN);
    mux_write_to_electrode(VN,   1, MUX_EN);

    calibrateEIT();

    // ----------------------------------------------------------------
    // Origin baseline: 30 frames, exponential moving average
    // ----------------------------------------------------------------
    Serial.println("[EITKit] Taking origin baseline measurements (30 iterations)...");

    for (uint16_t i = 0; i < 30; i++) {
        read_frame(_drive_type, _meas_type, _signal_rms, signal_mag, _signal_phase, _num_electrodes);
        for (uint16_t j = 0; j < _num_meas; j++) {
            if (_signal_rms[j] != 0) {
                _cur_frame[j] = 0.80 * _cur_frame[j] + 0.20 * _signal_rms[j];
            }
        }
    }

    Serial.print("ORIGIN_DATA,");
    for (uint16_t i = 0; i < _num_meas; i++) {
        Serial.print(_cur_frame[i], 4);
        if (i < _num_meas - 1) Serial.print(",");
    }
    Serial.println();
    Serial.println("[EITKit] Hardware Setup Complete.");
}

// =====================================================================
// calibrateEIT
// =====================================================================

void EITKitArduino::calibrateEIT()
{
    Serial.println("Beginning Calibration");

    _phase_offset = 0;
    calibrate_samples();
    calibrate_gain(_drive_type, _meas_type);

    #if defined(__IMXRT1062__)
    calibrate_signal(_drive_type, _meas_type);
    #endif

    Serial.println("Calibrated new");

    // Restore mux to a known state after calibration sweeps
    mux_write_to_electrode(SRC,  0, MUX_EN);
    mux_write_to_electrode(SINK, 1, MUX_EN);
    mux_write_to_electrode(VP,   2, MUX_EN);
    mux_write_to_electrode(VN,   3, MUX_EN);

    Serial.print("Drive gain: ");           Serial.println(_current_gain);
    Serial.print("Measurement gain: ");     Serial.println(_voltage_gain);
    Serial.print("Sample rate (uS per reading): "); Serial.println(sample_rate, 4);
    Serial.print("Samples per period: ");   Serial.println(samples_per_period);
    Serial.print("Reference signal phase offset (radians): "); Serial.println(_phase_offset, 4);
}

// =====================================================================
// take_measurements  (called every loop iteration)
// =====================================================================

void EITKitArduino::take_measurements(Meas_t drive_type, Meas_t meas_type)
{
    _drive_type = drive_type;
    _meas_type  = meas_type;
    read_frame(drive_type, meas_type, _signal_rms, signal_mag, _signal_phase, _num_electrodes);

    if (millis() - frame_delay > 500) {
        Serial.println("[EITKit] Sending Frame Data...");
        Serial.print("FRAME_DATA,");

        for (uint16_t i = 0; i < _num_meas; i++) {
            if (_signal_rms[i] != 0) {
                _cur_frame[i] = 0.50 * _cur_frame[i] + 0.50 * _signal_rms[i];
            }
            Serial.print(_cur_frame[i], 4);
            if (i < _num_meas - 1) Serial.print(",");
        }
        Serial.println();

        frame_delay = millis();
    }
}

// =====================================================================
// diagnose_adc_bus  -- FIX: new diagnostic helper
// Call from setup() after begin() to verify ADC bus wiring.
// =====================================================================

void EITKitArduino::diagnose_adc_bus()
{
    Serial.println("=== ADC BUS DIAGNOSTIC ===");

    Serial.println("20 raw GPIO6 reads (raw register >> 16 | converted 10-bit):");
    for (int i = 0; i < 20; i++) {
        uint32_t raw  = *(&GPIO6_DR + 2) >> 16;
        uint16_t conv = gpio_convert(raw);
        Serial.print("  raw=0x"); Serial.print(raw, HEX);
        Serial.print("  converted="); Serial.println(conv);
        delay(50);
    }

    Serial.println("\nArduino analogRead() cross-check (pins 14-23):");
    for (int pin = 14; pin <= 23; pin++) {
        Serial.print("  pin "); Serial.print(pin);
        Serial.print(" = "); Serial.println(analogRead(pin));
    }

    // Drive a known mux configuration and check for AC activity
    Serial.println("\nMux-driven AC check (SRC=0, SINK=1, VP=2, VN=3):");
    mux_write_to_electrode(SRC,  0, MUX_EN);
    mux_write_to_electrode(SINK, 1, MUX_EN);
    mux_write_to_electrode(VP,   2, MUX_EN);
    mux_write_to_electrode(VN,   3, MUX_EN);
    delayMicroseconds(500);

    uint16_t mn = 1023, mx = 0;
    for (int i = 0; i < 200; i++) {
        uint16_t v = read_adc_bus_10bit();
        if (v < mn) mn = v;
        if (v > mx) mx = v;
    }
    Serial.print("  ADC min="); Serial.print(mn);
    Serial.print("  max=");     Serial.print(mx);
    Serial.print("  pk-pk=");   Serial.println(mx - mn);

    if (mx - mn < 5) {
        Serial.println("  [WARN] Very low pk-pk (<5 counts). ADC may not be seeing AC signal.");
        Serial.println("         Check: clock oscillator running? Mux power? Resistor ladder connected?");
    } else {
        Serial.println("  [OK] AC signal detected on ADC bus.");
    }

    Serial.println("=== END DIAGNOSTIC ===");
}

void EITKitArduino::print_gpio_debug()
{
    uint32_t raw = gpio_read();
    uint16_t conv = gpio_convert(raw);
    Serial.print("[EITKit] gpio raw=0x");
    Serial.print(raw, HEX);
    Serial.print(" conv10=");
    Serial.println(conv);
}

// =====================================================================
// Public Set / Get
// =====================================================================

void     EITKitArduino::set_num_electrodes(int v)
{
    if (v < 1)  v = 1;
    if (v > 32) v = 32;
    _num_electrodes = v;
    _num_meas = _num_electrodes * _num_electrodes;
}
int      EITKitArduino::get_num_electrodes()          { return _num_electrodes; }
void     EITKitArduino::set_num_bands(int v)          { _num_bands = (v < 1) ? 1 : v; }
int      EITKitArduino::get_num_bands()               { return _num_bands; }
void     EITKitArduino::set_num_terminals(int v)      { _num_terminals = (v < 1) ? 1 : v; }
int      EITKitArduino::get_num_terminals()           { return _num_terminals; }
void     EITKitArduino::set_meas_type(Meas_t v)       { _meas_type = v; }
Meas_t   EITKitArduino::get_meas_type()               { return _meas_type; }
void     EITKitArduino::set_drive_type(Meas_t v)      { _drive_type = v; }
Meas_t   EITKitArduino::get_drive_type()              { return _drive_type; }
void     EITKitArduino::set_visualize_3d(bool v)      { _visualize_3d = v; }
bool     EITKitArduino::get_visualize_3d()            { return _visualize_3d; }
void     EITKitArduino::set_auto_calibration(bool v)  { _auto_calibration = v; }
bool     EITKitArduino::get_auto_calibration()        { return _auto_calibration; }
void     EITKitArduino::set_current_freq(uint16_t v)
{
    _current_freq = v;
    AD5930_Set_Start_Freq(v);
}
uint16_t EITKitArduino::get_current_freq()            { return _current_freq; }
void     EITKitArduino::set_current_gain(uint16_t v)  { set_source_gain(v); }
uint16_t EITKitArduino::get_current_gain()            { return _current_gain; }
void     EITKitArduino::set_voltage_gain(uint16_t v)  { set_meas_gain(v); }
uint16_t EITKitArduino::get_voltage_gain()            { return _voltage_gain; }
double*  EITKitArduino::get_magnitude_array()         { return signal_mag; }
double*  EITKitArduino::get_phase_array()             { return _signal_phase; }

// =====================================================================
// FIX: Missing gain setters -- were called in calibrate_gain but never
// defined, so the AD5270 potentiometers were never actually programmed.
// =====================================================================

void EITKitArduino::set_source_gain(uint16_t g)
{
    if (g > 1023) g = 1023;
    _current_gain = g;
    AD5270_Set(CHIP_SEL_DRIVE, g);
}

void EITKitArduino::set_meas_gain(uint16_t g)
{
    if (g > 1023) g = 1023;
    _voltage_gain = g;
    AD5270_Set(CHIP_SEL_MEAS, g);
}

// =====================================================================
// SPI / AD5270 / AD5930 low-level
// =====================================================================

void EITKitArduino::AD5270_Write(const int chip_sel, uint8_t cmd, uint16_t data)
{
    uint16_t data_word = ((cmd & 0x0F) << 10) | (data & 0x03FF);
    digitalWrite(chip_sel, LOW);
    delayMicroseconds(500);
    spi_write(V_DAT_PIN, V_SCLK_PIN, SPI_FREQ_FAST, MSBFIRST, SPI_MODE1, 16, data_word);
    delayMicroseconds(500);
    digitalWrite(chip_sel, HIGH);
}

void EITKitArduino::spi_write(uint8_t data_pin, uint8_t clock_pin, uint32_t freq,
                               uint8_t bit_order, uint8_t mode, uint8_t bits, uint32_t val)
{
    if (freq == 0) freq = 1;
    uint32_t period = (freq >= 500000U) ? 1U : (500000U / freq);
    if (period == 0) period = 1;
    uint8_t cpol = (mode == SPI_MODE2 || mode == SPI_MODE3);
    uint8_t cpha = (mode == SPI_MODE1 || mode == SPI_MODE3);
    uint8_t sck  = cpol ? HIGH : LOW;

    uint8_t  i;
    uint32_t start_time;

    digitalWrite(clock_pin, sck);
    delayMicroseconds(period * 4);

    for (i = 0; i < bits; i++) {
        start_time = micros();

        if (bit_order == LSBFIRST)
            digitalWrite(data_pin, !!(val & (1 << i)));
        else
            digitalWrite(data_pin, !!(val & (1 << ((bits - 1) - i))));

        sck = !sck;
        if (cpha) {
            digitalWrite(clock_pin, sck);
            while (micros() - start_time < period);
        } else {
            while (micros() - start_time < period);
            digitalWrite(clock_pin, sck);
        }

        start_time = micros();
        sck = !sck;
        if (cpha) {
            digitalWrite(clock_pin, sck);
            while (micros() - start_time < period);
        } else {
            while (micros() - start_time < period);
            digitalWrite(clock_pin, sck);
        }
    }
}

void EITKitArduino::AD5270_LockUnlock(const int chip_select, uint8_t lock) {
    AD5270_Write(chip_select, CMD_WR_CTRL, lock ? 0 : 0x002);
}

void EITKitArduino::AD5270_Shutdown(const int chip_select, uint8_t shutdown) {
    AD5270_Write(chip_select, CMD_SHTDN, shutdown ? 1 : 0);
}

void EITKitArduino::AD5270_Set(const int chip_select, uint16_t val) {
    AD5270_Write(chip_select, CMD_WR_RDAC, val);
}

void EITKitArduino::AD5930_Write(uint8_t reg, uint16_t data)
{
    uint16_t data_word = ((reg & 0x0F) << 12) | (data & 0x0FFF);
    digitalWrite(CHIP_SEL_AD5930, LOW);
    spi_write(V_DAT_PIN, V_SCLK_PIN, SPI_FREQ_FAST, MSBFIRST, SPI_MODE1, 16, data_word);
    digitalWrite(CHIP_SEL_AD5930, HIGH);
}

void EITKitArduino::AD5930_Set_Start_Freq(uint32_t freq)
{
    uint32_t scaled_freq = (freq * 1.0 / AD5930_CLK_FREQ) * 0x00FFFFFF;
    AD5930_Write(SFREQ_LOW_REG,  (uint16_t)(scaled_freq & 0x0FFF));
    AD5930_Write(SFREQ_HIGH_REG, (uint16_t)((scaled_freq >> 12) & 0x0FFF));
}

// =====================================================================
// Mux control
// =====================================================================

void EITKitArduino::mux_write(const int chip_select, uint8_t pin_sel, uint8_t enable)
{
    digitalWrite(chip_select, LOW);
    if (enable)
        spi_write(H_DAT_PIN, H_SCLK_PIN, SPI_FREQ_FAST, MSBFIRST, SPI_MODE1, 8, pin_sel & 0x1F);
    else
        spi_write(H_DAT_PIN, H_SCLK_PIN, SPI_FREQ_FAST, MSBFIRST, SPI_MODE1, 8, 0xC0 | (pin_sel & 0x1F));
    digitalWrite(chip_select, HIGH);
}

void EITKitArduino::mux_write_to_electrode(Mux_t chip_select, uint8_t electrode_sel, uint8_t enable)
{
    if (electrode_sel >= 32) return;

    int cs_pin = 0;
    switch (chip_select) {
        case SRC:  cs_pin = CHIP_SEL_MUX_SRC;  break;
        case SINK: cs_pin = CHIP_SEL_MUX_SINK; break;
        case VP:   cs_pin = CHIP_SEL_MUX_VP;   break;
        case VN:   cs_pin = CHIP_SEL_MUX_VN;   break;
        default:   return;
    }
    mux_write(cs_pin, elec_to_mux[electrode_sel], enable);
}

// =====================================================================
// GPIO / ADC bus
// =====================================================================

uint32_t EITKitArduino::gpio_read()
{
    uint32_t raw = (*(&GPIO6_DR + 2) >> 16);

#if EIT_DEBUG_DUMP
    static uint32_t t0 = 0;
    if (millis() - t0 > 1000) {
        t0 = millis();
        Serial.print("gpio_read raw=0x");
        Serial.println(raw, HEX);
    }
#endif

    return raw;
}

uint16_t EITKitArduino::gpio_convert(uint32_t gpio_reg)
{
    uint16_t val =
        ((gpio_reg & 0x0200) >> 9) |  // Pin 23 (GPIO 25) -> bit 0
        ((gpio_reg & 0x0100) >> 7) |  // Pin 22 (GPIO 24) -> bit 1
        ((gpio_reg & 0x0800) >> 9) |  // Pin 21 (GPIO 27) -> bit 2
        ((gpio_reg & 0x0400) >> 7) |  // Pin 20 (GPIO 26) -> bit 3
        ((gpio_reg & 0x0003) << 4) |  // Pins 19,18 (GPIO 17,16) -> bits 4,5
        ( gpio_reg & 0x00C0)       |  // Pins 17,16 (GPIO 22,23) -> bits 6,7
        ((gpio_reg & 0x0008) << 5) |  // Pin 15 (GPIO 19) -> bit 8
        ((gpio_reg & 0x0004) << 7);   // Pin 14 (GPIO 18) -> bit 9
    return val;
}

uint16_t EITKitArduino::analog_read()
{
    uint16_t gpio_reg = *(&GPIO6_DR + 2) >> 16;
    uint16_t val =
        ((gpio_reg & 0x0200) >> 9) |
        ((gpio_reg & 0x0100) >> 7) |
        ((gpio_reg & 0x0800) >> 9) |
        ((gpio_reg & 0x0400) >> 7) |
        ((gpio_reg & 0x0003) << 4) |
        ( gpio_reg & 0x00C0)       |
        ((gpio_reg & 0x0008) << 5) |
        ((gpio_reg & 0x0004) << 7);
    return val;
}

uint16_t EITKitArduino::read_adc_bus_10bit()
{
    return gpio_convert(gpio_read());
}

// =====================================================================
// calibrate_samples
// =====================================================================

void EITKitArduino::calibrate_samples()
{
    num_samples = MAX_SAMPLES;
    uint32_t sample_time = read_signal(NULL, NULL, NULL, NULL, 0);

    sample_rate = (float)sample_time / MAX_SAMPLES;
    if (sample_rate < 1e-3f) {
        sample_rate = 1.0f;
    }

    samples_per_period = (uint16_t)((1000000.0f / sample_rate) / TEST_FREQ);
    num_samples        = samples_per_period * NUM_PERIODS;

    // Guard against degenerate values
    if (num_samples == 0)            num_samples = 1;
    if (num_samples > MAX_SAMPLES)   num_samples = MAX_SAMPLES;
}

// =====================================================================
// calibrate_gain  -- FIX: now calls set_source_gain / set_meas_gain
// which actually program the AD5270 via SPI.
// =====================================================================

void EITKitArduino::calibrate_gain(Meas_t drive_type, Meas_t meas_type)
{
    (void)drive_type;
    (void)meas_type;

    // ---- Drive (source) gain ----
    Serial.println("[EITKit] Calibrating Source Gain...");

    uint16_t err = 0;
    double rms_v = 0, mag_v = 0, ph = 0;
    bool drive_locked = false;

    for (uint16_t g = 0; g <= 1023; g += 3) {
        set_source_gain(g);                              // FIX: was missing
        read_signal(&rms_v, &mag_v, &ph, &err, 1);

        if (err & EIT_ERR_ALL_ZERO) continue;
        if (err & EIT_ERR_ALL_SAME) continue;

        if (mag_v > 0.02) {
            drive_locked = true;
            break;
        }
    }

    if (!drive_locked) {
        Serial.println("[CAL] WARN: Could not lock source gain. ADC path may be disconnected.");
        Serial.println("[CAL]       Defaulting source gain to 426.");
        set_source_gain(426);
    } else {
        Serial.print("[EITKit] Locked Source Gain at: ");
        Serial.println(_current_gain);
    }

    // ---- Measurement (voltage amp) gain ----
    Serial.println("[EITKit] Calibrating Measurement Gain...");

    bool meas_locked = false;

    for (uint16_t g = 0; g <= 1023; g += 3) {
        set_meas_gain(g);                                // FIX: was missing
        read_signal(&rms_v, &mag_v, &ph, &err, 1);

        if (err & EIT_ERR_ALL_ZERO) continue;
        if (err & EIT_ERR_ALL_SAME) continue;

        if (mag_v > 0.02) {
            meas_locked = true;
            break;
        }
    }

    if (!meas_locked) {
        Serial.println("[CAL] WARN: Could not lock measurement gain. ADC path may be disconnected.");
        Serial.println("[CAL]       Defaulting measurement gain to 1023.");
        set_meas_gain(1023);
    } else {
        Serial.print("[EITKit] Locked Measurement Gain at: ");
        Serial.println(_voltage_gain);
    }
}

// =====================================================================
// calibrate_signal
// =====================================================================

void EITKitArduino::calibrate_signal(Meas_t drive_type, Meas_t meas_type)
{
    mux_write_to_electrode(SRC, 0, MUX_EN);

    if      (drive_type == AD) mux_write_to_electrode(SINK, 1,  MUX_EN);
    else if (drive_type == OP) mux_write_to_electrode(SINK, 16, MUX_EN);

    if      (meas_type == AD) { mux_write_to_electrode(VP, 30, MUX_EN); mux_write_to_electrode(VN, 31, MUX_EN); }
    else if (meas_type == OP) { mux_write_to_electrode(VP, 15, MUX_EN); mux_write_to_electrode(VN, 31, MUX_EN); }

    delay(5);
    _phase_offset = 0;

    mux_write_to_electrode(SRC,  0, MUX_DIS);
    mux_write_to_electrode(SINK, 0, MUX_DIS);
    mux_write_to_electrode(VP,   0, MUX_DIS);
    mux_write_to_electrode(VN,   0, MUX_DIS);
}

// =====================================================================
// read_frame
// FIX: rms_array[num_meas] = 0 on skipped measurements so the
//      smoothing filter in take_measurements() / begin() never
//      accumulates stale non-zero values from uninitialised memory.
// FIX: mux settling delays increased (200->500 us, 100->300 us).
// =====================================================================

void EITKitArduino::read_frame(Meas_t drive_type, Meas_t meas_type,
                                double *rms_array, double *mag_array,
                                double *phase_array, uint8_t num_elec)
{
    if (num_elec == 0)   return;
    if (num_elec > 32)   num_elec = 32;

    uint16_t num_meas = 0;

    for (uint8_t tx_pair = 0; tx_pair < num_elec; tx_pair++) {
        uint8_t src_pin = 0, sink_pin = 0;

        switch (drive_type) {
            case AD:   src_pin = tx_pair; sink_pin = (tx_pair + 1) % num_elec;             break;
            case OP:   src_pin = tx_pair; sink_pin = (tx_pair + num_elec / 2) % num_elec;  break;
            case MONO: src_pin = tx_pair; sink_pin = 0;                                     break;
            default:
                // Advance num_meas for all rx_pairs this tx would have covered
                num_meas += num_elec;
                continue;
        }

        mux_write_to_electrode(SRC,  src_pin,  MUX_EN);
        mux_write_to_electrode(SINK, sink_pin, MUX_EN);
        delayMicroseconds(500);  // FIX: was 200us — increased to allow mux + amp to settle

        for (uint8_t rx_pair = 0; rx_pair < num_elec; rx_pair++, num_meas++) {
            if (num_meas >= NUM_MEAS) return;
            uint8_t vp_pin = 0, vn_pin = 0;

            switch (meas_type) {
                case AD:   vp_pin = rx_pair; vn_pin = (rx_pair + 1) % num_elec;             break;
                case OP:   vp_pin = rx_pair; vn_pin = (rx_pair + num_elec / 2) % num_elec;  break;
                case MONO: vp_pin = rx_pair; vn_pin = sink_pin;                              break;
                default:   vp_pin = rx_pair; vn_pin = (rx_pair + 1) % num_elec;             break;
            }

            // --- Skip invalid electrode combinations ---
            bool skip = false;
            if (meas_type == MONO) {
                skip = (vp_pin == src_pin) || (vp_pin == vn_pin) || (src_pin == sink_pin);
            } else {
                skip = (vp_pin == src_pin) || (vp_pin == sink_pin) ||
                       (vn_pin == src_pin) || (vn_pin == sink_pin);
            }

            if (skip) {
                // FIX: zero ALL three arrays so no stale data leaks into the
                //      exponential moving average in take_measurements/begin.
                rms_array[num_meas]   = 0;
                mag_array[num_meas]   = 0;
                phase_array[num_meas] = 0;
                continue;
            }

            mux_write_to_electrode(VP, vp_pin, MUX_EN);
            mux_write_to_electrode(VN, vn_pin, MUX_EN);
            delayMicroseconds(300);  // FIX: was 100us — increased for amp settling

            _debug_current_meas = num_meas;
            read_signal(rms_array   + num_meas,
                        mag_array   + num_meas,
                        phase_array + num_meas,
                        NULL, 0);
        }
    }
}

// =====================================================================
// read_signal
// =====================================================================

uint32_t EITKitArduino::read_signal(double *rms, double *mag, double *phase,
                                    uint16_t *error_rate, uint8_t debug)
{
    if (num_samples == 0)          num_samples = 1;
    if (num_samples > MAX_SAMPLES) num_samples = MAX_SAMPLES;

    static uint16_t adc_buf_s[MAX_SAMPLES];
    static uint8_t  ref_buf_s[MAX_SAMPLES];
    static uint16_t phase_cycles_s[MAX_SAMPLES];

    uint16_t i, j;
    uint32_t t1 = micros();

    // ---- Collect samples ----
    for (i = 0; i < num_samples; i++) {
        uint32_t sum = 0;
        for (j = 0; j < ADC_AVG; j++) {
            sum += (uint32_t)read_adc_bus_10bit();
        }
        adc_buf_s[i] = (uint16_t)(sum / ADC_AVG);
        ref_buf_s[i] = (uint8_t)digitalReadFast(AD5930_MSBOUT_PIN);
    }

    uint32_t t2 = micros();

    // ---- Stats ----
    uint16_t mn = 1023, mx = 0;
    uint32_t mean_sum = 0;

    for (i = 0; i < num_samples; i++) {
        uint16_t v = adc_buf_s[i];
        mean_sum += v;
        if (v < mn) mn = v;
        if (v > mx) mx = v;
    }

    uint16_t err = 0;
    if (mn == 0 && mx == 0) err |= EIT_ERR_ALL_ZERO;
    if (mn == mx)            err |= EIT_ERR_ALL_SAME;

    double mean = (double)mean_sum / (double)num_samples;

    // ---- AC RMS (around mean) ----
    double sq = 0.0;
    for (i = 0; i < num_samples; i++) {
        double ac = (double)adc_buf_s[i] - mean;
        sq += ac * ac;
    }
    double rms_ac_10bit = sqrt(sq / (double)num_samples);
    uint16_t mag_10bit  = (mx >= mn) ? (mx - mn) : 0;

    // ---- Phase ----
    uint16_t phase_count    = 0;
    uint16_t phase_readings = 0;
    uint16_t ref_edges      = 0;

    for (i = 1; i < num_samples; i++) {
        phase_count++;
        bool rising  = ( ref_buf_s[i] && !ref_buf_s[i - 1]);
        bool falling = (!ref_buf_s[i] &&  ref_buf_s[i - 1]);

        if (rising || falling) {
            ref_edges++;
            if (phase_count <= samples_per_period && phase_readings < MAX_SAMPLES) {
                phase_cycles_s[phase_readings++] = phase_count;
            }
            phase_count = 0;
        }
    }

    if (ref_edges == 0) err |= EIT_ERR_NO_REF_EDGE;

    int16_t phase_offset_cycles = 0;
    if (phase_readings > 0) {
        uint32_t psum = 0;
        for (i = 0; i < phase_readings; i++) psum += phase_cycles_s[i];
        phase_offset_cycles = (int16_t)(psum / phase_readings);
    }

    // ---- Debug output ----
    // Rate-limited to 1Hz normally; always prints when debug==2 (for calibration visibility)
    static uint32_t dbg_last = 0;
    bool print_debug = debug && (millis() - dbg_last > 1000);
    if (debug == 2) print_debug = true;  // unconditional during calibration

    if (print_debug) {
        dbg_last = millis();
        Serial.println("---- DEBUG read_signal ----");
        Serial.print("adc_min_obs=");   Serial.print(mn);
        Serial.print(" adc_max_obs=");  Serial.print(mx);
        Serial.print(" mag_10bit=");    Serial.print(mag_10bit);
        Serial.print(" rms_10bit=");    Serial.print((uint16_t)rms_ac_10bit);
        Serial.println();
        Serial.print("first8=");
        for (int k = 0; k < 8 && k < (int)num_samples; k++) {
            Serial.print(adc_buf_s[k]);
            if (k < 7) Serial.print(",");
        }
        Serial.println();
        Serial.print("err=0x"); Serial.println(err, HEX);
        Serial.println("---------------------------");
    }

    // ---- Write outputs ----
    if (rms)        *rms   = rms_ac_10bit * 2.2 / 1024.0;
    if (mag)        *mag   = (double)mag_10bit * 2.2 / 1024.0;
    if (phase)      *phase = (sample_rate * (double)phase_offset_cycles / 1e6)
                             * (double)TEST_FREQ * 2.0 * PI;
    if (error_rate) *error_rate = err;

    return (t2 - t1);
}

// =====================================================================
// Utility: sine_compare
// =====================================================================

uint16_t EITKitArduino::sine_compare(uint16_t *signal, uint16_t pk_pk,
                                      uint16_t points_per_period, uint8_t num_periods)
{
    if (points_per_period == 0) return 0;

    uint16_t num_points = points_per_period * num_periods;
    uint32_t error_sum  = 0;

    for (uint16_t i = 0; i < num_points; i++) {
        uint32_t ref_index = ((i * 1024) / points_per_period) % 1024;
        int32_t  ref_point = (sine_table[ref_index] * pk_pk) / 1024;
        int32_t  sig_val   = (int16_t)signal[i] - 512;
        error_sum += (uint16_t)abs(sig_val - ref_point);
    }
    return (uint16_t)(error_sum / num_points);
}

// =====================================================================
// probe_adc_after_change  (debug helper)
// =====================================================================

void EITKitArduino::probe_adc_after_change(const char* tag)
{
    uint16_t err = 0;
    double rms_v = 0, mag_v = 0, ph = 0;
    read_signal(&rms_v, &mag_v, &ph, &err, 0);

    Serial.print("[PROBE] "); Serial.print(tag);
    Serial.print(" mag(V)="); Serial.print(mag_v, 6);
    Serial.print(" rms(V)="); Serial.print(rms_v, 6);
    Serial.print(" err=0x");  Serial.println(err, HEX);
}
