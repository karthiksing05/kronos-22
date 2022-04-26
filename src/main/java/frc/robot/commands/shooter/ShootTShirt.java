// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.TitanButton;
import frc.robot.subsystems.Shooter;

public class ShootTShirt extends CommandBase {

  private final Shooter shooter;

  private final TitanButton solenoidBtn;
  private final TitanButton rotateBtn;

  public ShootTShirt(Shooter shooter, TitanButton solenoidBtn, TitanButton rotateBtn) {

    this.shooter = shooter;

    this.solenoidBtn = solenoidBtn;
    this.rotateBtn = rotateBtn;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (this.solenoidBtn.isPressed()) {
      shooter.setBarrelSolenoid(true);
      // todo wait a predetermined delay?
      shooter.setBarrelSolenoid(false);
    }
    if (this.rotateBtn.isPressed()) {
      // todo change color depending on whatever color the tape is and also change
      do {
        shooter.setRotateSpeed(0.3);
      }
      while (!(shooter.getColor().red > 0.3));
      shooter.setRotateSpeed(0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
