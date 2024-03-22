// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class ActivateIntake extends Command {
  private Intake intake;
  private double input;
  public  ActivateIntake(Intake intake, double input) {
    this.intake = intake;
    this.input = input;
    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intake.setIntakeSpeed(input);
  }

  @Override
  public void end(boolean interrupted) {
    intake.setIntakeSpeed(0);
  }

  @Override
  public boolean isFinished() {
    if (input > 0 && intake.noteCollected()){
      return true;
    }
    return false;
  }
}
