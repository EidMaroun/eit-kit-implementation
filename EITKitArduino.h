/*
  EITKitArduino.h - Library for EIT-kit Sensing Board.
  Will be released into the public domain.
*/
#ifndef EITKitArduino_h
#define EITKitArduino_h

#include <Arduino.h>
#include "SPI.h"

#if !defined(__IMXRT1062__)
  #error "EITKitArduino: This library now supports Teensy 4.x only (__IMXRT1062__)."
#endif

#define MUX_EN             1
#define MUX_DIS            0

#define NUM_ELECTRODES     32
#define NUM_MEAS           (NUM_ELECTRODES * NUM_ELECTRODES)
#define MAX_ELECTRODES     64

// AD5930 register addresses
#define CTRL_REG           0x00
#define NUM_INCR_REG       0x01
#define DFREQ_LOW_REG      0x02
#define DFREQ_HIGH_REG     0x03
#define TIME_INCR_REG      0x04
#define TIME_BURST_REG     0x08
#define SFREQ_LOW_REG      0x0C
#define SFREQ_HIGH_REG     0x0D

typedef enum { AD, OP, MONO } Meas_t;
typedef enum { SRC, SINK, VP, VN } Mux_t;

#define SPI_FREQ_FAST      500000
#define SPI_FREQ_SLOW      500000

// Vertical control bus (AD5930 + AD5270)
#define V_DAT_PIN          11  // V_DAT
#define V_SCLK_PIN         13  // V_SCLK

// Horizontal control bus (ADG731 mux boards)
#define H_DAT_PIN          26  // H_DAT
#define H_SCLK_PIN         27  // H_SCLK

// Schematic oscillator is 50 MHz (IQXO-793)
#define AD5930_CLK_FREQ    50000000
#define TEST_FREQ          40000
#define NUM_PERIODS        5
#define ADC_AVG            7
#define MAX_SAMPLES        2000

// AD5270 commands
#define CMD_WR_RDAC        0x01
#define CMD_RD_RDAC        0x02
#define CMD_ST_RDAC        0x03
#define CMD_RST            0x04
#define CMD_RD_MEM         0x05
#define CMD_RD_ADDR        0x06
#define CMD_WR_CTRL        0x07
#define CMD_RD_CTRL        0x08
#define CMD_SHTDN          0x09

// Chip selects / control pins
#define CHIP_SEL_AD5930    3   // CS_AD5930
#define CHIP_SEL_DRIVE     0   // CS_DRIVE
#define CHIP_SEL_MEAS      1   // CS_MEAS
#define CHIP_SEL_MUX_SRC   24  // CS_MUX1
#define CHIP_SEL_MUX_SINK  28  // CS_MUX2
#define CHIP_SEL_MUX_VP    30  // CS_MUX3
#define CHIP_SEL_MUX_VN    32  // CS_MUX4

#define AD5930_MSBOUT_PIN  6   // MSB
#define AD5930_INT_PIN     5   // INT
#define AD5930_CTRL_PIN    4   // CTRL
#define AD5930_STANDBY_PIN 2   // STBY

// read_signal error bitmask (also used externally)
#define EIT_ERR_ALL_ZERO     0x0001
#define EIT_ERR_ALL_SAME     0x0002
#define EIT_ERR_NO_REF_EDGE  0x0004

class EITKitArduino
{
public:
  EITKitArduino();
  EITKitArduino(int num_electrodes, int num_bands, int num_terminals,
                Meas_t drive_type, Meas_t meas_type, bool bluetooth_communication);

  void begin();
  void take_measurements(Meas_t drive_type, Meas_t meas_type);

  // Hardware bring-up diagnostic — call after begin()
  void diagnose_adc_bus();

  // Explicit GPIO snapshot print (gpio_read no longer auto-prints)
  void print_gpio_debug();

  void set_num_electrodes(int num_electrodes);
  int  get_num_electrodes();
  void set_num_bands(int num_bands);
  int  get_num_bands();
  void set_num_terminals(int num_terminals);
  int  get_num_terminals();

  void   set_meas_type(Meas_t meas_type);
  Meas_t get_meas_type();
  void   set_drive_type(Meas_t drive_type);
  Meas_t get_drive_type();

  void set_visualize_3d(bool visualize_3d);
  bool get_visualize_3d();
  void set_auto_calibration(bool auto_calibration);
  bool get_auto_calibration();

  void     set_current_freq(uint16_t current_freq);
  uint16_t get_current_freq();
  void     set_current_gain(uint16_t current_gain);
  uint16_t get_current_gain();
  void     set_voltage_gain(uint16_t voltage_gain);
  uint16_t get_voltage_gain();

  double* get_magnitude_array();
  double* get_phase_array();

private:
  const uint8_t elec_to_mux[MAX_ELECTRODES] = {
     9, 10, 11,  8,  7,  6,  5,  4,  3,  2,  1,  0, 12, 13, 14, 15,
    31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16,
     0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15,
    31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16
  };

  uint16_t _debug_current_meas = 0;

  float    sample_rate        = 0;
  uint16_t samples_per_period = 0;
  uint16_t num_samples        = 1;
  double   ref_signal_mag     = 0;

  uint16_t adc_buf[MAX_SAMPLES];

  uint8_t  pin_num  = 0;
  uint16_t rheo_val = 1023;

  int _num_electrodes = 32;
  int _num_meas       = _num_electrodes * _num_electrodes;
  int _num_bands      = 1;
  int _num_terminals  = 4;

  Meas_t _drive_type = AD;
  Meas_t _meas_type  = AD;

  bool _visualize_3d     = false;
  bool _auto_calibration = true;

  double _signal_rms[NUM_MEAS]   = {0};
  double _signal_phase[NUM_MEAS] = {0};
  double signal_mag[NUM_MEAS]    = {0};
  double _cur_frame[NUM_MEAS]    = {0};

  double   _phase_offset = 0;
  uint32_t frame_delay   = 0;

  uint16_t _current_amp  = 0;
  uint16_t _current_freq = 0;
  uint16_t _current_gain = 0;
  uint16_t _voltage_gain = 0;

  bool serial_communication     = true;
  bool _bluetooth_communication = false;

  void calibrateEIT();
  void calibrate_samples();
  void calibrate_gain(Meas_t drive_type, Meas_t meas_type);
  void calibrate_signal(Meas_t drive_type, Meas_t meas_type);

  // FIX: declared here — were missing from original header
  void set_source_gain(uint16_t g);
  void set_meas_gain(uint16_t g);

  void AD5270_Write(const int chip_select, uint8_t cmd, uint16_t data);
  void AD5270_LockUnlock(const int chip_select, uint8_t lock);
  void AD5270_Shutdown(const int chip_select, uint8_t shutdown);
  void AD5270_Set(const int chip_select, uint16_t val);

  void AD5930_Write(uint8_t reg, uint16_t data);
  void AD5930_Set_Start_Freq(uint32_t freq);

  void spi_write(uint8_t data_pin, uint8_t clock_pin, uint32_t freq,
                 uint8_t bit_order, uint8_t mode, uint8_t bits, uint32_t val);

  uint16_t analog_read();

  void mux_write_to_electrode(Mux_t chip_select, uint8_t electrode_sel, uint8_t enable);
  void mux_write(const int chip_select, uint8_t pin_sel, uint8_t enable);

  uint32_t gpio_read();
  uint16_t gpio_convert(uint32_t gpio_reg);
  uint16_t read_adc_bus_10bit();

  void probe_adc_after_change(const char* tag);

  uint16_t sine_compare(uint16_t *signal, uint16_t pk_pk,
                        uint16_t points_per_period, uint8_t num_periods);

  void read_frame(Meas_t drive_type, Meas_t meas_type,
                  double *rms_array, double *mag_array,
                  double *phase_array, uint8_t num_elec);

  uint32_t read_signal(double *rms, double *mag, double *phase,
                       uint16_t *error_rate, uint8_t debug);
};

#endif
