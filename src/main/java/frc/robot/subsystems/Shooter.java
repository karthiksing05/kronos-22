// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import com.revrobotics.ColorSensorV3;
import frc.robot.motor.TitanSRX;
import edu.wpi.first.math.MathUtil;

public class Shooter extends SubsystemBase {

  private final Compressor compressor;
  private final Solenoid barrelSolenoid;

  private final TitanSRX tiltMotor;
  private final TitanSRX rotateMotor;

  private final ColorSensorV3 colorSensor;

  public Shooter(Compressor compressor, Solenoid barrelSolenoid, TitanSRX tiltMotor, TitanSRX rotateMotor, ColorSensorV3 colorSensor) {

    this.compressor = compressor;
    this.barrelSolenoid = barrelSolenoid;

    this.tiltMotor = tiltMotor;
    this.rotateMotor = rotateMotor;

    this.colorSensor = colorSensor;

  }

  // Pneumatics basic functions
  public Solenoid getBarrelSolenoid() {
    return barrelSolenoid;
  }

  public void setBarrelSolenoid(boolean bool) {
    barrelSolenoid.set(bool);
  }

  public boolean getPressureSwitchValue() {
    return this.compressor.getPressureSwitchValue();
  }

  // Tilt Motor basic functions
  public TitanSRX getTiltMotor() {
    return tiltMotor;
  }

  public void setTiltSpeed(double speed) {
    rotateMotor.set(MathUtil.clamp(speed, -1, 1));
  }

  public void stopTiltMotor() {
    setTiltSpeed(0);
  }

  public void brakeTiltMotor() {
    tiltMotor.brake();
  }

  // Rotate Motor basic functions
  public TitanSRX getRotateMotor() {
    return rotateMotor;
  }

  public void setRotateSpeed(double speed) {
    rotateMotor.set(MathUtil.clamp(speed, -1, 1));
  }

  public void stopRotateMotor() {
    setRotateSpeed(0);
  }

  public void brakeRotateMotor() {
    rotateMotor.brake();
  }

  // Color Sensor basic functions
  public Color getColor() {
    return colorSensor.getColor();
  }

  

}
