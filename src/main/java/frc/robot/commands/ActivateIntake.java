// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class ActivateIntake extends Command {
  private Intake intake;
  public ActivateIntake(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intake.setIntakeSpeed(1);
  }

  @Override
  public void end(boolean interrupted) {
    intake.setIntakeSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
