#include <simpletools.h>
#include <adcDCpropab.h>
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

static int sensor_stack[128 + 40];
static volatile SENSOR_STATE SensorState;
static volatile IMU_STATE ImuState;

#ifdef USE_SPI_ADC
static uint16_t SPIReadADC(uint8_t channel)
{
    uint8_t ii;
    uint32_t result;
    uint32_t setup;

    //Setting up pins

    // In case pin was already been low, we put it high
    // so we can initiate communication after setting up pins
    set_output(SPI_CS, 1);
    set_output(SPI_DATA, 0);
    set_output(SPI_CLK, 0);
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

    pulse_out(SPI_CLK, 1); //Empty clock, for sampling
    pulse_out(SPI_CLK, 1); //Device returns low, NULL bit, we ignore it...
    input(SPI_DATA);

    // read ADC result 12 bit
    result = 0;
    for(ii = 0; ii < 12; ++ii) 
    {
        // We are sending pulse, clock signal, to ADC, because on falling edge it will return data...
        pulse_out(SPI_CLK, 1);
        // Shifting bit to left, to make room for current one...
        result <<= 1;
        result = result | (get_state(SPI_DATA) & 0x01);
    }
    high(SPI_CS);
}
#endif

static uint16_t I2CReadADC(uint8_t channel)
{
    const uint8_t CHANNEL[8] = {0x8C,0xCC,0x9C,0xDC,0xAC,0xEC,0xBC,0xFC};
                                 // Constant configuration data
    uint8_t a2d_valueH;
    uint8_t a2d_valueL;

    // Note: Passing 0 means no register
    I2C_ByteWrite(AD7828_ADDR, 0, CHANNEL[channel]);
    pause(100);
    I2C_ByteRead(AD7828_ADDR, 0, &a2d_valueH);    
    I2C_ByteRead(AD7828_ADDR, 0, &a2d_valueL);
    
    return (a2d_valueH << 8) + a2d_valueL;
}

static float PropReadADC(uint8_t channel)
{
    adc_init(PROP_A2D_CS, PROP_A2D_SCL, PROP_A2D_DO, PROP_A2D_DI);
    return adc_volts(channel);
}

static float ReadAnalogIr(uint8_t index)
{
    uint16_t result;
#ifdef USE_SPI_ADC
    result = SPIReadADC(index);
#endif
    result = I2CReadADC(index);
    
    float voltage = ((result + 1) * IR_REF_VOLTAGE * 100) / ADC_RESOLUTION;
    
    //http://home.roboticlab.eu/en/examples/sensor/ir_distance
    #define CONST_A 5461.0
    #define CONST_B -17.0
    #define CONST_K 2.0
        
    if (voltage + CONST_B <= 0)
    {
        return -1;
    }
 
    return CONST_A / (voltage + CONST_B) - CONST_K;
}    

static uint8_t ReadDigitalIr(uint8_t index)
{
    uint32_t pulse;
    
    set_output(DIGITAL_IR_PIN_START + index, 1);
    usleep(10);
    
    // units of pulse are microseconds
    pulse = pulse_in(DIGITAL_IR_PIN_START + index, 0);
    
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

static float ReadUltrasonic(uint8_t addr)
{
    // Set pins directions
    set_direction(MUX_ADDR_START, 1);
    set_direction(MUX_ADDR_START + 1, 1);
    set_direction(MUX_ADDR_START+ 2, 1);
    set_direction(MUX_ADDR_START + 3, 1);
    set_direction(TRIG_PIN, 1);
    set_direction(ECHO_PIN, 0);

    // Set the mux based on sensor index
    set_outputs(MUX_ADDR_END, MUX_ADDR_START, (int) addr); 

    // Pulse the trigger pin
    low(TRIG_PIN);
    pulse_out(TRIG_PIN, 10);

    // Read in echo pulse
    uint32_t pulse = pulse_in(ECHO_PIN, 1);

    return pulse/58.0;    
}

static void PollAnalogIRSensors()
{
    uint8_t ii;
    for (ii = 0; ii < sizeof(SensorState.analog_ir); ++ii)
    {
        SensorState.analog_ir[ii] = 1000.0;//ReadAnalogIr(ii);
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
    for (ii = 0; ii < 7; ++ii)
    {
        SensorState.ultrasonic[ii] = ReadUltrasonic(ii);
    }
    SensorState.ultrasonic[ii] = 100.0;
    for (ii = 8; ii < 13; ++ii)
    {
        SensorState.ultrasonic[ii] = ReadUltrasonic(ii);
    }
    SensorState.ultrasonic[ii++] = 100;
    SensorState.ultrasonic[ii++] = 100;
    SensorState.ultrasonic[ii++] = 100;
}

static void PollImuSensor()
{
    ReadImu(&ImuState);
}

static void PollMotionSensor()
{
    SensorState.motion_detected = 0;
}

static void PollSensors(void *par)
{
    uint8_t ii;
    while (1)
    {
        //PollAnalogIRSensors();
        // Note: There is an issue when this call is enabled.  It could be stack related or it could be that there is no
        // sensor attached.  Will need to debug when digital IR sensors are added.
        //PollDigitalIRSensors(); 
        PollUltrasonicSensors();
        //PollImuSensor();
        //PollMotionSensor();
    }
}

void GetSensorState(SENSOR_STATE* state)
{
    memcpy(state, &SensorState, sizeof(SensorState));
}

void GetImuState(IMU_STATE* state)
{
    memcpy(state, &ImuState, sizeof(ImuState));
}

void SensorsStart()
{
    uint8_t ii;
    
    memset(&SensorState, 0, sizeof(SensorState));
    for (ii = 0; ii < sizeof(SensorState.analog_ir); ++ii)
    {
        SensorState.analog_ir[ii] = 1000.0;
    }
    
    cogstart(&PollSensors, NULL, sensor_stack, sizeof(sensor_stack));
}

