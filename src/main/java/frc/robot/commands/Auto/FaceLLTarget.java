// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PIDConstants;
import frc.robot.Robot.LimelightHelpers;
import frc.robot.subsystems.Drivetrain;

//TODO: test and tune
public class FaceLLTarget extends Command {
  private Drivetrain drivetrain;
  private PIDController directionController;

  public FaceLLTarget(Drivetrain drivetrain ) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    directionController = new PIDController(
      PIDConstants.Turn.p,
      PIDConstants.Turn.i,
      PIDConstants.Turn.d
    );
    directionController.setSetpoint(0);
    directionController.setTolerance(0.5);
  }

  @Override
  public void execute() {
    double calculation = directionController.calculate(LimelightHelpers.getTX(""));
    drivetrain.tankDrive(-calculation, calculation);
  }

  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return directionController.atSetpoint();
  }
}
