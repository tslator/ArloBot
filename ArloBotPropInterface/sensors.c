#include <simpletools.h>
#include <adcDCpropab.h>
#include <math.h>
#include "i2cbus.h"
#include "sensors.h"
#include "imu.h"
#include "utils.h"
#include "hwconfig.h"

#define IR_REF_VOLTAGE (5.0)
#define ADC_RESOLUTION (4096)

#define SENSITIVITY_ACC   0.06103515625   // LSB/mg
#define SENSITIVITY_MAG   0.00048828125   // LSB/Ga

// Added to get rid of warning.  Not sure why the include of simpletools.h didn't resolve this.
void adc_init(int csPin, int sclPin, int doPin,int diPin);
float adc_volts(int channel);

// Note: This is the maximum stack size that still works.  Any bigger and nothing happens.
static int sensor_stack[40 + 136];
static volatile SENSOR_STATE SensorState;
#if 0
static volatile IMU_STATE ImuState;
#endif

#ifdef USE_SPI_ADC
static uint16_t SPIReadADC(uint8_t channel)
{
    uint8_t ii;
    uint32_t result;
    uint32_t setup;

    //Setting up pins

    // In case pin was already been low, we put it high
    // so we can initiate communication after setting up pins
    set_output(SPI_CS, HIGH);
    set_output(SPI_DATA, LOW);
    set_output(SPI_CLK, LOW);
    low(SPI_CS);   // Active chip select by setting pin low

    // Sending configuration to device
    setup = channel | 0b11000;
    for(ii = 0; ii < 5; ++ii) 
    {
        pulse_out(SPI_CLK, 1);
        if ((setup & 0b10000) == 0b10000)
        {
            high(SPI_DATA);
        }            
        else
        {
            low(SPI_DATA); // is MSB != 0
        }            
        setup <<= 1;  // shift left
    }

    pulse_out(SPI_CLK, HIGH); //Empty clock, for sampling
    pulse_out(SPI_CLK, HIGH); //Device returns low, NULL bit, we ignore it...
    input(SPI_DATA);

    // read ADC result 12 bit
    result = 0;
    for(ii = 0; ii < 12; ++ii) 
    {
        // We are sending pulse, clock signal, to ADC, because on falling edge it will return data...
        pulse_out(SPI_CLK, HIGH);
        // Shifting bit to left, to make room for current one...
        result <<= 1;
        result = result | (get_state(SPI_DATA) & 0x01);
    }
    high(SPI_CS);
}
#endif

static float ReadAnalogIr(uint8_t index)
{
    uint16_t result;
#ifdef USE_SPI_ADC
    result = SPIReadADC(index);
#else
    // Two AD7828 are supported:
    //     Channels 0 - 7 are assigned to AD7828_ADDR_0
    //     Channels 8 - 15 are assigned to AD7828_ADDR_1
    uint8_t addr = index < (NUM_ANALOG_IR_SENSORS/2) ? AD7828_ADDR_0 : AD7828_ADDR_1;
    result = I2C_ReadADC(addr + AD7827_WRITE_MODIFIER, addr + AD7827_READ_MODIFIER, index % NUM_ANALOG_IR_SENSORS);
#endif
    
    float voltage = ((result + 1) * IR_REF_VOLTAGE * 100) / ADC_RESOLUTION;
    
    return 27.86 * pow(voltage, -1.15);
}    

#ifdef DIGITAL_IR_SUPPORTED
static uint8_t ReadDigitalIr(uint8_t index)
{
    uint32_t pulse;
    
    set_output(DIGITAL_IR_PIN_START + index, OUTPUT);
    usleep(10);
    
    // units of pulse are microseconds
    pulse = pulse_in(DIGITAL_IR_PIN_START + index, LOW);
    
    // White surfaces reflect more light than black, so, when directed towards a white surface, 
    // the capacitor will discharge faster than it would when pointed towards a black surface.
    
    // pulses longer than 3 seconds mean nothing was found
    // shorter pulses mean no detection
    // longer pulses mean detection
    // might need some calibration to determine the threshold
    
    // Convert to milliseconds
    pulse /= 1000;
    
    if (pulse <= 500 || pulse >= 3000)
    {
        return 0;
    }
    
    return 1;
}
#endif

static float ReadUltrasonic(uint8_t addr)
{
    // Set pins directions
    set_direction(MUX_ADDR_START, OUTPUT);
    set_direction(MUX_ADDR_START + 1, OUTPUT);
    set_direction(MUX_ADDR_START+ 2, OUTPUT);
    set_direction(MUX_ADDR_START + 3, OUTPUT);
    set_direction(TRIG_PIN, OUTPUT);
    set_direction(ECHO_PIN, INPUT);

    // Set the mux based on sensor index
    set_outputs(MUX_ADDR_END, MUX_ADDR_START, (int) addr); 

    // Pulse the trigger pin
    low(TRIG_PIN);
    pulse_out(TRIG_PIN, 10);

    // Read in echo pulse
    uint32_t pulse = pulse_in(ECHO_PIN, HIGH);

    return pulse/58.0;    
}

static void PollAnalogIRSensors()
{
    uint8_t ii;
    uint8_t addr = AD7828_ADDR_0;
    
    for (ii = 0; ii < NUM_ANALOG_IR_SENSORS; ++ii)
    {
        if (ii > 8)
        {
          addr = AD7828_ADDR_1;
        }          
        uint16_t result = I2C_ReadADC(addr + AD7827_WRITE_MODIFIER, addr + AD7827_READ_MODIFIER, ii);
        float voltage = ((result + 1) * IR_REF_VOLTAGE * 100) / ADC_RESOLUTION;
        SensorState.analog_ir[ii] = 27.86 * pow(voltage, -1.15);
    }        
}
    
static void PollDigitalIRSensors()
{
    uint8_t ii;
    for (ii = 0; ii < sizeof(SensorState.digital_ir); ++ii)
    {
        SensorState.digital_ir[ii] = 0;//ReadDigitalIr(ii);
    }        
}
    
static void PollUltrasonicSensors()
{   
    uint8_t ii;
    for (ii = 0; ii < sizeof(SensorState.ultrasonic); ++ii)
    {
        SensorState.ultrasonic[ii] = ReadUltrasonic(ii);
    }
}

#ifdef IMU_SUPPORTED
static void PollImuSensor()
{
    ReadImu(&ImuState);
}
#endif

static void PollMotionSensor()
{
    SensorState.motion_detected = 0;//get_state(MOTION_DETECT);
}
#ifdef MOTOR_POWER_SUPPORTED
static void PollMotorPower()
{
    uint8_t ii;
    
    // Wow, the following commented code would not work (crashed the board)
    // But, the code after works just fine!!! WTF!!!
    //SensorState.left_motor_voltage = adc_volts(LEFT_MOTOR_VOLTAGE);
    //SensorState.right_motor_voltage = adc_volts(RIGHT_MOTOR_VOLTAGE);
    //SensorState.left_motor_current = adc_volts(LEFT_MOTOR_CURRENT);
    //SensorState.right_motor_current = adc_volts(RIGHT_MOTOR_CURRENT);
    
    adc_init(PROP_A2D_CS, PROP_A2D_SCL, PROP_A2D_DO, PROP_A2D_DI);
    
    for (ii = 0; ii < 4; ++ii)
    {
        float value = adc_volts(ii);
        switch (ii)
        {
            case 0:
                SensorState.left_motor_voltage = value;
                break;
            case 1:
                SensorState.right_motor_voltage = value;
                break;
            case 2:
                SensorState.left_motor_current = value;
                break;
            case 3:
                SensorState.right_motor_current = value;
                break;
        }
    }
}
#endif

static void PollSensors(void *par)
{
    while (1)
    {
        PollAnalogIRSensors();
        #ifdef DIGITAL_IR_SUPPORTED
        // Note: There is an issue when this call is enabled.  It could be stack related or it could be that there is no
        // sensor attached.  Will need to debug when digital IR sensors are added.
        //PollDigitalIRSensors(); 
        #endif
        //PollUltrasonicSensors();
        #ifdef IMU_SUPPORTED
        //PollImuSensor();
        #endif
        //PollMotionSensor();
        #ifdef MOTOR_POWER_SUPPORTED
        //PollMotorPower();
        #endif
    }
}

void GetSensorState(SENSOR_STATE* state)
{
    memcpy(state, &SensorState, sizeof(SensorState));
}

#if 0
void GetImuState(IMU_STATE* state)
{   
    memcpy(state, &ImuState, sizeof(ImuState));
}
#endif

void SensorsStart()
{
    uint8_t ii;
    
    memset(&SensorState, 0, sizeof(SensorState));
    
    cogstart(&PollSensors, NULL, sensor_stack, sizeof(sensor_stack));
}