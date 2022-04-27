// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.motor.TitanSRX;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import com.revrobotics.ColorSensorV3;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TankDrive;

import frc.robot.commands.drivetrain.DriveTeleop;
import frc.robot.commands.drivetrain.DriveTeleopArcade;
import frc.robot.commands.shooter.ShootTShirt;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.OI;
import frc.robot.sensors.TitanButton;

import frc.robot.RobotMap;

public class RobotContainer {

  public RobotContainer() {

    configureButtonBindings();

  }

  private void configureButtonBindings() {

  }

//  public Command getAutonomousCommand() {
//    // An ExampleCommand will run in autonomous
//    return
//  }
}
