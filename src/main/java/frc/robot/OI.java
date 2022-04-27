package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class OI {

	private Joystick leftJoystick;
	private Joystick rightJoystick;

	private static final double DEADBAND_PERCENTAGE = 0.1; // TODO Adjust this as necessary

	public OI() {
		initialize();
	}

	private void initialize() {
		this.leftJoystick = new Joystick(RobotMap.LEFT_JOYSTICK);
		this.rightJoystick = new Joystick(RobotMap.RIGHT_JOYSTICK);
	}

	public Joystick getLeftJoystick() {
		return this.leftJoystick;
	}
	public Joystick getRightJoystick() {
		return this.rightJoystick;
	}

	public double getLeftJoyY() {
		return deadband(-leftJoystick.getY(), DEADBAND_PERCENTAGE);
	}

	public double getRightJoyY() {
		return deadband(-rightJoystick.getY(), DEADBAND_PERCENTAGE);
	}

	// This method is for arcade drive :)
	public double getRightJoyX() {
		return deadband(-rightJoystick.getX(), DEADBAND_PERCENTAGE);
	}

	public double deadband(double value, double deadband) {
		if (-deadband <= value && value <= deadband) {
			value = 0;
		} else if (value > deadband) {
			value -= deadband;
			value *= (1 + deadband);
		} else if (value < -deadband) {
			value += deadband;
			value *= (1 + deadband);
		}
		return value;
	}

}
