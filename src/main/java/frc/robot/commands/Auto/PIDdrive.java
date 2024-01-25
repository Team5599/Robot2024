// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainMechanism;
import frc.robot.subsystems.Drivetrain;

public class PIDdrive extends Command {
  private PIDController controller;
  private Drivetrain drivetrain;
  private double setpoint;

  public PIDdrive(Drivetrain drivetrain, double setpoint) {
    this.drivetrain = drivetrain;
    this.setpoint = setpoint;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    //TODO: test and tune these values, currently arbitrary
    controller.setSetpoint(setpoint);
    controller = new PIDController(1, 0, 0.5);
    drivetrain.ResetEncoders();
    controller.setTolerance(0.2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double measurement = (drivetrain.LeftPosition() + drivetrain.RightPosition())/2;
    measurement *= 2 * Math.PI * DrivetrainMechanism.wheelRadius;
    controller.calculate(measurement);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}
