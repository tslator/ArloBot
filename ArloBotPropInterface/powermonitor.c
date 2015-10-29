


static int throttleStatus;

    // Send a regular "status" update to ROS including information that does not need to be refreshed as often as the odometry.
    throttleStatus = throttleStatus + 1;
    if (throttleStatus > 9) {
        // Check Motor Power
        double leftMotorPower = 4.69;
        double rightMotorPower = 4.69;
        #ifdef hasMotorPowerMonitorCircuit
        // The input numbers for adc_init are printed right on the Activity Board.
        adc_init(21, 20, 19, 18);
        leftMotorPower = adc_volts(LEFT_MOTOR_ADC_PIN);
        rightMotorPower = adc_volts(RIGHT_MOTOR_ADC_PIN);
        #endif
        //dprint(term, "s\t%d\t%d\t%d\t%d\t%d\t%d\t%.2f\t%.2f\t%d\t%d\n", safeToProceed, safeToRecede, Escaping, abd_speedLimit, abdR_speedLimit, minDistanceSensor, leftMotorPower, rightMotorPower, cliff, floorO);
        throttleStatus = 0;
    }
    #ifdef debugModeOn
    //dprint(term, "DEBUG: %d %d %d %d %d\n", ignoreProximity, ignoreCliffSensors, ignoreIRSensors, ignoreFloorSensors, pluggedIn);
    #endif
