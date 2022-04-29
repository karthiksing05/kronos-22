// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;

import frc.robot.motor.TitanSRX;

import com.revrobotics.ColorSensorV3;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TankDrive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drivetrain.DriveTeleop;
import frc.robot.commands.drivetrain.DriveTeleopArcade;
import frc.robot.commands.shooter.ShootTShirt;
import frc.robot.commands.autonomous.DoNothing;

import frc.robot.sensors.TitanButton;

public class RobotContainer {

  // motors
  public TitanSRX leftMotor;
  public TitanSRX rightMotor;
  public TitanSRX rotateMotor;
  public TitanSRX tiltMotor;

  // pneumatics
  public Compressor compressor;
  public Solenoid barrelSolenoid;

  // sensors
  public ColorSensorV3 colorSensor;

  // subsystems
  public TankDrive drivetrain;
  public Shooter shooter;

  // oi + buttons
  public OI oi;

  public TitanButton btnRotateBarrel;
  public TitanButton btnShoot;
  public TitanButton btnBarrelUp;
  public TitanButton btnBarrelDown;

  // teleop commands
  public DriveTeleop driveTeleopCommand;
  public DriveTeleopArcade arcadeDriveTeleopCommand;
  public ShootTShirt shootTShirtCommand;

  // auto commands;
  public DoNothing emptyAuto;

  public RobotContainer() {

    // motors
    leftMotor = new TitanSRX(RobotMap.LEFT_SRX, true);
    rightMotor = new TitanSRX(RobotMap.RIGHT_SRX, false);
    rotateMotor = new TitanSRX(RobotMap.ROTATE_SRX, false);
    tiltMotor = new TitanSRX(RobotMap.TILT_SRX, false);

    // pneumatics
    compressor = new Compressor(RobotMap.PNEUMATICS_HUB_ID, PneumaticsModuleType.CTREPCM);
    barrelSolenoid = new Solenoid(RobotMap.PNEUMATICS_HUB_ID, PneumaticsModuleType.CTREPCM, RobotMap.BARREL_SOLENOID);

    // sensors
    colorSensor = new ColorSensorV3(RobotMap.COLOR_SENSOR_PORT);

    // subsystems
    drivetrain = new TankDrive(leftMotor, rightMotor);
    shooter = new Shooter(compressor, barrelSolenoid, tiltMotor, rotateMotor, colorSensor);

    // oi + buttons
    oi = new OI();
    btnRotateBarrel = new TitanButton(oi.getLeftJoystick(), OI.ROTATE_BTN_ID);
    btnShoot = new TitanButton(oi.getRightJoystick(), OI.TILT_BTN_ID);
    btnBarrelUp = new TitanButton(oi.getRightJoystick(), OI.BARREL_UP_BTN_ID);
    btnBarrelDown = new TitanButton(oi.getLeftJoystick(), OI.BARREL_DOWN_BTN_ID);

    // teleop commands
    driveTeleopCommand = new DriveTeleop(drivetrain, oi::getLeftJoyY, oi::getRightJoyY, false);
    arcadeDriveTeleopCommand = new DriveTeleopArcade(drivetrain, oi::getLeftJoyY, oi::getRightJoyX, false);
    shootTShirtCommand = new ShootTShirt(shooter, btnShoot, btnRotateBarrel);

    // auto commands
    emptyAuto = new DoNothing();

    configureButtonBindings();

  }

  private void configureButtonBindings() {

    btnRotateBarrel.toggleWhenPressed(shootTShirtCommand);
    btnShoot.toggleWhenPressed(shootTShirtCommand);
    btnBarrelUp.whenPressed(new InstantCommand(() -> shooter.setTiltSpeed(0.5))); // todo adjust speeds for these two
    btnBarrelDown.whenPressed(new InstantCommand(() -> shooter.setTiltSpeed(-0.5)));
    btnBarrelUp.whenReleased(new InstantCommand(() -> shooter.setTiltSpeed(0)));
    btnBarrelDown.whenReleased(new InstantCommand(() -> shooter.setTiltSpeed(0)));

  }

  public Command getAutonomousCommand() {
    return emptyAuto;
  }
}