#include "gyro.h"


// For Gyroscope:
/*
   NOTE About Gyro module:
   Currently this code makes NO use of the gyro. The Odometry from the ArloBot's wheel encoders is excellent!
   The only thing I do with the gyro is send the data to ROS.
   At some point a ROS node could use that data to detect a serious issue, like the robot being picked up or being stuck.
   As it is though, gmapping, AMCL, etc. work very well off of the Odometry with using the gyro data.
   */
static unsigned char i2cAddr = 0x69;       //I2C Gyro address
//L3G4200D register addresses & commads.
//See device datasheet section 7 for more info.
static unsigned char devId = 0x0f;        //Device ID
static unsigned char ctrl1 = 0x20;        //Control reg1
static unsigned char cfg1 = 0b00011111;   //100 hz, 25 cutoff, power up, axes enabled
static unsigned char ctrl2 = 0x21;
static unsigned char ctrl3 = 0x22;
static unsigned char cfg3 = 0b00001000;    //Enable data poling (I2_DRDY)
static unsigned char ctrl4 = 0x23;
static unsigned char cfg4 = 0b10000000;    //Block until read, big endian
static unsigned char status = 0x27;
static unsigned char xL = 0x28;            //Reg for x low byte - Next 5 bytes xH, yL, yH, zL, xH
static unsigned char reply;                //Single byte reply
static char xyz[6];                        //XYZ dat array
static int gyroXvel, gyroYvel, gyroZvel;                       //Axis variables
static i2c *bus;                           //Declare I2C bus
// Create a cog for polling the Gyro
static void pollGyro(void *par); // Use a cog to fill range variables with ping distances
static int gyrostack[128]; // If things get weird make this number bigger!

// We need this even if there is no Gyro. Just pass 0.0 if it doesn't exist.
// Otherwise I would have to modify the propeller_node.py too.
static double gyroHeading = 0.0;

static void pollGyro(void *par) 
{
    while (1) {
        int ready = 0;                    //Wait until ready
        while (!ready) {
            i2c_in(bus, i2cAddr, status, 1, &ready, 1);
            ready = 1 & (ready >>= 3);
        }

        for (int i = 0; i < 6; i++)        //Get axis bytes
        {
            int regAddr = xL + i;
            i2c_in(bus, i2cAddr, regAddr, 1, &xyz[i], 1);
        }

        //Bytes to int in Degrees Per Second (dps)
        //"Dividing by 114 reduces noise"
        // http://www.parallax.com/sites/default/files/downloads/27911-L3G4200D-Gyroscope-Application-Note.pdf
        // 1 radian/second [rad/s] = 57.2957795130824 degree/second [°/s]
        gyroXvel = (int) (short) ((xyz[1] << 8) + xyz[0]) / 114; // Perhaps use later to detect tipping?
        gyroYvel = (int) (short) ((xyz[3] << 8) + xyz[2]) / 114; // Perhaps use later to detect tipping?
        gyroZvel = (int) (short) ((xyz[5] << 8) + xyz[4]) / 114;

        // If Gyro is running at 100Hz then time between readings should be 10 milliseconds
        double deltaGyroHeading = 0.01 * gyroZvel * 2; // I'm not sure why I have to multiply by two, but I do.
        deltaGyroHeading = deltaGyroHeading * PI / 180.0; // Convert to Radians

        // Discard small variations when motors are not running to eliminate stationary drift
        // Maybe this should be ANY time that speedLeft == speedRight? Then straight lines would stay straight, since
        // ActivityBot appears to travel VERY good straight lines, but they seem to wobble in RVIZ at the moment.
        if (speedLeft == 0 && speedRight == 0) {
            if (deltaGyroHeading < 0.01) { // But accept large changes in case the robot is bumped or moved. Adjust as needed
                deltaGyroHeading = 0.0;
            }
        }

        gyroHeading += deltaGyroHeading;

        // limit heading to -Pi <= heading < Pi
        if (gyroHeading > PI) {
            gyroHeading -= 2.0 * PI;
        } else {
            if (gyroHeading <= -PI) {
                gyroHeading += 2.0 * PI;
            }
        }

        //pause(250); // Pause between reads, or do we need this? Should we read faster? The !ready loop should handle the Gyro's frequency right?
    }
}


void GYRO_Init()
{
    gyroXvel = 0;
    gyroYvel = 0;
    gyroZvel = 0;
    gyroHeading = 0.0;
}

void GYRO_Start()
{
    // Initialize Gyro in the main program
    bus = i2c_newbus(GYRO_SCL_PIN, GYRO_SDA_PIN, 0);        //New I2C bus SCL = Pin 1, SDA = Pin 0
    int n;
    n = i2c_out(bus, i2cAddr, ctrl3, 1, &cfg3, 1);
    n += i2c_out(bus, i2cAddr, ctrl4, 1, &cfg4, 1);
    n += i2c_out(bus, i2cAddr, ctrl1, 1, &cfg1, 1);
    // Make sure Gyro initialized and stall if it did not.
    if (n != 9) {
        print("Bytes should be 9, but was %d,", n);
        while (1); // This should just TELL ROS that there is no gyro available instead of stalling the program,
        // TODO:
        // OR have ROS tell us if we HAVE a gyro and only start this if we think we do.
        // That way the program works with or without a gyro
    }
    // Start Gyro polling in another cog
    cogstart(&pollGyro, NULL, gyrostack, sizeof gyrostack);
}

void GYRO_SetHeading(double value)
{
    gyroHeading = value;
}

void GYRO_GetHeading()
{
    return gyroHeading;
}
