// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motor.TitanSRX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.motor.Encoder;

public class TankDrive extends SubsystemBase {

  private final TitanSRX leftMotor;
  private final TitanSRX rightMotor;

  public static final double MAX_SPEED = 1;
  public static final double DRIVETRAIN_INCHES_PER_PULSE = 18.85 / 2048f; // inches per pulse

  public TankDrive(TitanSRX leftTalonSRX, TitanSRX rightTalonSRX) {
    this.leftMotor = leftTalonSRX;
    this.rightMotor = rightTalonSRX;

    leftTalonSRX.configOpenloopRamp(1);
    rightTalonSRX.configOpenloopRamp(1);
  }

  // Set wheel speeds individually
  public void set(double leftTSpeed, double rightTSpeed) {
    leftTSpeed = MathUtil.clamp(leftTSpeed, -1, 1);
    rightTSpeed = MathUtil.clamp(rightTSpeed, -1, 1);
    leftMotor.set(leftTSpeed);
    rightMotor.set(rightTSpeed);
  }

  // Stops the motors from moving
  public void stop() {
    this.set(0, 0);
  }

  public void brake() {
    leftMotor.brake();
    rightMotor.brake();
  }

  public void coast() {
    leftMotor.coast();
    rightMotor.coast();
  }

  public TitanSRX getLeftMotor() {
    return leftMotor;
  }

  public TitanSRX getRightMotor() {
    return rightMotor;
  }

  public Encoder getLeftEncoder() {
    return leftMotor.getEncoder();
  }

  public Encoder getRightEncoder() {
    return rightMotor.getEncoder();
  }

  public void enableBrownoutProtection() {
    leftMotor.enableBrownoutProtection();
    rightMotor.enableBrownoutProtection();
  }

  public void disableBrownoutProtection() {
    leftMotor.disableBrownoutProtection();
    rightMotor.disableBrownoutProtection();
  }

  public double[] getSpeeds() {
    return new double[]{leftMotor.getSpeed(), rightMotor.getSpeed()};
  }

  public boolean isRunning() {
    return leftMotor.getSpeed() != 0 || rightMotor.getSpeed() != 0;
  }


}
