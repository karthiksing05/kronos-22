// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motor.TitanFXV2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.motor.Encoder;

public class TankDrive extends SubsystemBase {

  private final TitanFXV2 leftMotor;
  private final TitanFXV2 rightMotor;

  public static final double MAX_SPEED = 1;
  public static final double DRIVETRAIN_INCHES_PER_PULSE = 18.85 / 2048f; // inches per pulse

  public TankDrive(TitanFXV2 leftTalonFX, TitanFXV2 rightTalonFX) {
    this.leftMotor = leftTalonFX;
    this.rightMotor = rightTalonFX;
    resetEncoders();

    leftTalonFX.configOpenloopRamp(1);
    rightTalonFX.configOpenloopRamp(1);
  }

  @Override
  public void periodic(){

    // Debugging the DriveTrain
    SmartDashboard.putBoolean("Drivetrain Running", isRunning());
    SmartDashboard.putNumber("Left DT Encoder", getLeftEncoderPosition());
    SmartDashboard.putNumber("Right DT Encoder", getRightEncoderPosition());

  }

  // Returns current wheel speeds
  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(leftMotor.getSensorCollection().getIntegratedSensorVelocity() * 10 * DRIVETRAIN_INCHES_PER_PULSE * 0.0254, -rightMotor.getSensorCollection().getIntegratedSensorVelocity() * 10 * DRIVETRAIN_INCHES_PER_PULSE * 0.0254);
  }

  // Get left encoder position
  public double getLeftEncoderPosition() {
    return getLeftMotor().getSensorCollection().getIntegratedSensorPosition();
  }

  // Get right encoder position
  public double getRightEncoderPosition() {
    return -getRightMotor().getSensorCollection().getIntegratedSensorPosition();
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

  public void resetEncoders() {
    this.leftMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);
    this.rightMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);
  }

  public TitanFXV2 getLeftMotor() {
    return leftMotor;
  }

  public TitanFXV2 getRightMotor() {
    return rightMotor;
  }

  public Encoder getLeftEncoder() {
    return leftMotor.getEncoder();
  }

  public Encoder getRightEncoder() {
    return rightMotor.getEncoder();
  }

  public double getAverageEncoderDistance() {
    return (getLeftMotor().getSensorCollection().getIntegratedSensorPosition() + -getRightMotor().getSensorCollection().getIntegratedSensorPosition()) / 2.0;
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
