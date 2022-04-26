// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.motor.Filter;
import frc.robot.subsystems.TankDrive;

import java.util.function.DoubleSupplier;

public class DriveTeleopArcade extends CommandBase {

  private final TankDrive dt;
  private DoubleSupplier throttleInput, steeringInput;

  private Filter leftFilter, rightFilter;
  private boolean filterEnabled;

  public DriveTeleopArcade(TankDrive dt, DoubleSupplier throttleInput, DoubleSupplier steeringInput, boolean filterEnabled) {
    this.dt = dt;
    this.throttleInput = throttleInput;
    this.steeringInput = steeringInput;
    this.filterEnabled = filterEnabled;

    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.leftFilter = new Filter(0.5);
    this.rightFilter = new Filter(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double steering = steeringInput.getAsDouble();
    double throttle = throttleInput.getAsDouble();
    double rpower =  throttle + steering;
    double lpower = throttle - steering;

    this.dt.set(lpower, rpower);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.dt.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
