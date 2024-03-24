// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Auto.SetIntakeAngle;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class SourceIntake extends Command {
  private Intake intake;
  private Shooter shooter;

  public SourceIntake(Intake intake, Shooter shooter) {
    this.intake = intake;
    this.shooter = shooter;
    addRequirements(intake,shooter);
  }

  @Override
  public void initialize() {
    intake.setIntakeSpeed(-0.4);
    shooter.setShooterSpeed(-0.15);
  }

  @Override
  public void execute() {
    intake.setIntakeSpeed(0);
  }

  @Override
  public void end(boolean interrupted) {
    intake.setIntakeSpeed(0);
    shooter.setShooterSpeed(0);
  }

  @Override
  public boolean isFinished() {
    if (intake.noteCollected()){
      return true;
    }
    return false;
  }
}
