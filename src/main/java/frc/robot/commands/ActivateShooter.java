// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ActivateShooter extends Command {
  private Shooter shooter;
  public ActivateShooter(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    shooter.setShooterSpeed(0.325);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setShooterSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
