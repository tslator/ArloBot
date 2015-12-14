#include <simpletools.h>

#include "safety.h"

// Mapping for sensors
// Index 0 - 15 are Ultrasonic sensors (see hwconfig.h)
#define ULTRASONIC_START (0)
#define ULTRASONIC_END (15)

//   Index 0 - 7 are front sensors
//       Index 0 - 4 are lower deck
//       Index 5 - 9 are mid deck
#define NUM_ULTRASONIC_FRONT (8)
#define ULTRASONIC_FRONT_START (ULTRASONIC_START)
#define ULTRASONIC_FRONT_END (ULTRASONIC_FRONT_START + NUM_ULTRASONIC_FRONT - 1)

#define NUM_ULTRASONIC_FRONT_LOWER (5)
#define ULTRASONIC_FRONT_LOWER_START (ULTRASONIC_FRONT_START)
#define ULTRASONIC_FRONT_LOWER_END (ULTRASONIC_FRONT_LOWER_START + NUM_ULTRASONIC_FRONT_LOWER - 1)

#define NUM_ULTRASONIC_FRONT_MID (3)
#define ULTRASONIC_FRONT_MID_START (ULTRASONIC_FRONT_LOWER_END + 1)
#define ULTRASONIC_FRONT_MID_END (ULTRASONIC_FRONT_MID_START + NUM_ULTRASONIC_FRONT_MID - 1)

//   Index 8 - 15 are rear sensors
//       Index 8 - 12 are lower deck
//       Index 13 - 15 are mid deck
#define NUM_ULTRASONIC_REAR (8)
#define ULTRASONIC_REAR_START (ULTRASONIC_START + NUM_ULTRASONIC_FRONT)
#define ULTRASONIC_REAR_END (ULTRASONIC_REAR_START + NUM_ULTRASONIC_REAR - 1)

#define NUM_ULTRASONIC_REAR_LOWER (5)
#define ULTRASONIC_REAR_LOWER_START (ULTRASONIC_REAR_START)
#define ULTRASONIC_REAR_LOWER_END (ULTRASONIC_REAR_LOWER_START + NUM_ULTRASONIC_REAR_LOWER - 1)

#define NUM_ULTRASONIC_REAR_MID (3)
#define ULTRASONIC_REAR_MID_START (ULTRASONIC_REAR_LOWER_END + 1)
#define ULTRASONIC_REAR_MID_END (ULTRASONIC_REAR_MID_START + NUM_ULTRASONIC_REAR_MID - 1)

// Index 0 - 7 are Analog IR sensors
#define ANALOG_IR_START (0)
#define ANALOG_IR_END (7)

//   Index 0 - 3 are front sensors - all are on underside of top deck
//   Index 4 - 7 are rear sensors - all are on underside of top deck
#define NUM_ANALOG_IR_FRONT (4)
#define ANALOG_IR_FRONT_START (ANALOG_IR_START)
#define ANALOG_IR_FRONT_END (ANALOG_IR_FRONT_START + NUM_ANALOG_IR_FRONT - 1)

#define NUM_ANALOG_IR_REAR (4)
#define ANALOG_IR_REAR_START (ANALOG_IR_FRONT_END + 1)
#define ANALOG_IR_REAR_END (ANALOG_IR_REAR_START + NUM_ANALOG_IR_REAR - 1)

// Index 0 - 5 are Digital IR sensors
#define DIGITAL_IR_START (0)
#define DIGITAL_IR_END (5)

#define NUM_DIGITAL_IR_FRONT (3)
#define DIGITAL_IR_FRONT_START (DIGITAL_IR_START)
#define DIGITAL_IR_FRONT_END (DIGITAL_IR_FRONT_START + NUM_DIGITAL_IR_FRONT - 1)

#define NUM_DIGITAL_IR_REAR (3)
#define DIGITAL_IR_REAR_START (DIGITAL_IR_FRONT_END + 1)
#define DIGITAL_IR_REAR_END (DIGITAL_IR_REAR_START + NUM_DIGITAL_IR_REAR - 1)

// Cliff Sensors
// Note: Some (actually all) of the IR sensors are allocated to function as cliff sensors.  The difference between a "distance" sensor and a "cliff" sensor
// if the actual distance checked.  The "cliff" sensor distance value is further than the "distance" sensor value.
#define NUM_CLIFF_FRONT (NUM_ANALOG_IR_FRONT)
#define CLIFF_FRONT_START (ANALOG_IR_FRONT_START)
#define CLIFF_FRONT_END (ANALOG_IR_FRONT_END)

#define NUM_CLIFF_REAR (NUM_ANALOG_IR_REAR)
#define CLIFF_REAR_START (ANALOG_IR_REAR_START)
#define CLIFF_REAR_END (ANALOG_IR_REAR_END)

// Floor Obstacle Sensors
// As it relates to safety, the digital IR sensors can be used to detect floor objects
#define NUM_FLOOR_OBSTACLE_FRONT (NUM_DIGITAL_IR_FRONT)
#define FLOOR_OBSTACLE_FRONT_START (DIGITAL_IR_FRONT_START)
#define FLOOR_OBSTACLE_FRONT_END (DIGITAL_IR_FRONT_END)

#define NUM_FLOOR_OBSTACLE_REAR (NUM_DIGITAL_IR_REAR)
#define FLOOR_OBSTACLE_REAR_START (DIGITAL_IR_REAR_START)
#define FLOOR_OBSTACLE_REAR_END (DIGITAL_IR_REAR_END)


#define ULTRASONIC_MIN_SAFE_DISTANCE (6.0) // in centimeters
#define ANALOG_IR_MIN_SAFE_DISTANCE (6.0)  // in centimeters
#define CLIFF_MIN_SAFE_DISTANCE (50.0)     // in centimeters
#define MAX_DISTANCE (1000.0)              // in centimeters


static void CheckDetect(uint8_t* sensors, uint8_t start, uint8_t end, uint32_t* result)
{
    uint8_t ii;
    
    *result = 0;
    
    for (ii = start; ii <= end; ++ii)
    {
        if (sensors[ii])
        {
            *result |= 1 << ii;
        }
    }
}

static void CheckDistance(float* sensors, uint8_t start, uint8_t end, float min_safe_dist, float* sensor_dist, uint32_t* result)
{
    uint8_t ii;
    float min_dist = MAX_DISTANCE;
    
    *result = 0;
    
    for (ii = start; ii <= end; ++ii)
    {
        if (sensors[ii] < min_safe_dist)
        {
            *result |= 1 << ii;
        }
        if (sensors[ii] < min_dist)
        {
            min_dist = sensors[ii];
        }
    }
}

static void CheckCliffSensors(SENSOR_STATE* sensor_state, SAFETY_STATE* safety_state)
{
    uint32_t front_result = 0;
    uint32_t rear_result = 0;
    float sensor_dist = 0.0;
    
    // Assume the most restrictive state
    safety_state->cliff_detected = 1;
    
    CheckDistance(sensor_state->analog_ir, CLIFF_FRONT_START, CLIFF_FRONT_END, CLIFF_MIN_SAFE_DISTANCE, &sensor_dist, &front_result);
    CheckDistance(sensor_state->analog_ir, CLIFF_REAR_START, CLIFF_REAR_END, CLIFF_MIN_SAFE_DISTANCE, &sensor_dist, &rear_result);

    if (!front_result && !rear_result)
    {
        safety_state->cliff_detected = 0;
    }
}

static void CheckFloorSensors(SENSOR_STATE* sensor_state, SAFETY_STATE* safety_state)
{
    uint32_t front_result;
    uint32_t rear_result;
    
    // Assume the most restrictive state
    safety_state->floor_obstacle_detected = 1;
    
    CheckDetect(sensor_state->digital_ir, FLOOR_OBSTACLE_FRONT_START, FLOOR_OBSTACLE_FRONT_END, &front_result);
    CheckDetect(sensor_state->digital_ir, FLOOR_OBSTACLE_REAR_START, FLOOR_OBSTACLE_REAR_END, &rear_result);
    
    if (!front_result && !rear_result)
    {
        safety_state->floor_obstacle_detected = 0;
    }
}

static void CheckDistanceSensors(SENSOR_STATE* sensor_state, SAFETY_STATE* safety_state)
{
    uint32_t front_us_result;
    uint32_t front_ir_result;
    uint32_t rear_us_result;
    uint32_t rear_ir_result;
    float us_sensor_dist = 0.0;
    float ir_sensor_dist = 0.0;
    
    // Assume the most restrictive state
    safety_state->safe_to_proceed = 0;
    safety_state->safe_to_recede = 0;
    safety_state->min_distance_sensor = MAX_DISTANCE;
    
    CheckDistance(sensor_state->ultrasonic, ULTRASONIC_FRONT_LOWER_START, ULTRASONIC_FRONT_LOWER_END, ULTRASONIC_MIN_SAFE_DISTANCE, &us_sensor_dist, &front_us_result);
    if (us_sensor_dist < safety_state->min_distance_sensor)
    {
        safety_state->min_distance_sensor = us_sensor_dist;
    }
    CheckDistance(sensor_state->analog_ir, ANALOG_IR_FRONT_START, ANALOG_IR_FRONT_END, ANALOG_IR_MIN_SAFE_DISTANCE, &ir_sensor_dist, &front_ir_result);
    if (ir_sensor_dist < safety_state->min_distance_sensor)
    {
        safety_state->min_distance_sensor = ir_sensor_dist;
    }
    CheckDistance(sensor_state->ultrasonic, ULTRASONIC_REAR_LOWER_START, ULTRASONIC_REAR_LOWER_END, ULTRASONIC_MIN_SAFE_DISTANCE, &us_sensor_dist, &rear_us_result);
    if (us_sensor_dist < safety_state->min_distance_sensor)
    {
        safety_state->min_distance_sensor = us_sensor_dist;
    }
    CheckDistance(sensor_state->analog_ir, ANALOG_IR_REAR_START, ANALOG_IR_REAR_END, ANALOG_IR_MIN_SAFE_DISTANCE, &ir_sensor_dist, &rear_ir_result);
    if (ir_sensor_dist < safety_state->min_distance_sensor)
    {
        safety_state->min_distance_sensor = ir_sensor_dist;
    }

    if (!front_us_result && !front_ir_result)
    {
        safety_state->safe_to_proceed = 1;
    }
    
    if (!rear_us_result && !rear_ir_result)
    {
        safety_state->safe_to_recede = 1;
    }
}

void UpdateSafety(SENSOR_STATE* sensor_state, SAFETY_STATE* safety_state)
{
    // The purpose of safety is to evaluate various sensors for "dangerous" environments
    // and to the safety state appropriately.  Safety does not take any action other than
    // setting the safety state.
    
    if (safety_state->ignore_proximity || 
        (safety_state->ignore_dist_sensors && safety_state->ignore_cliff_sensors && safety_state->ignore_floor_sensors)
        )
    {
        safety_state->safe_to_proceed = 1;
        safety_state->safe_to_recede = 1;
        safety_state->cliff_detected = 0;
        safety_state->floor_obstacle_detected = 0;
    }
    else
    {
        if (!safety_state->ignore_dist_sensors)
        {
            CheckDistanceSensors(sensor_state, safety_state);
        }
        
        if (!safety_state->ignore_cliff_sensors)
        {
            CheckCliffSensors(sensor_state, safety_state);
        }
        
        if (!safety_state->ignore_floor_sensors)
        {
            CheckFloorSensors(sensor_state, safety_state);
        }
        
        if (safety_state->cliff_detected || safety_state->floor_obstacle_detected)
        {
            safety_state->safe_to_proceed = 0;
            safety_state->safe_to_recede = 0;
        }
    }
}