package frc.robot;
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import edu.wpi.first.wpilibj.I2C;

public final class RobotMap {

	// TODO Figure out what the appropriate ids are for each mechanism on the robot

	// CAN IDs
	public static final int PNEUMATICS_HUB_ID = 15;
	public static final int LEFT_SRX = 1;
	public static final int RIGHT_SRX = 2;
	public static final int ROTATE_SRX = 3;
	public static final int TILT_SRX = 4;

	// PCM IDs
	public static final int BARREL_SOLENOID = 1;

	// Sensor IDs
	public static I2C.Port COLOR_SENSOR_PORT = I2C.Port.kOnboard;;

	// Joysticks
	public static final int LEFT_JOYSTICK = 1;
	public static final int RIGHT_JOYSTICK = 0;

}
