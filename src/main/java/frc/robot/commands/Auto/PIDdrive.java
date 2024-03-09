// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class PIDdrive extends Command {
  private PIDController controller;
  private Drivetrain drivetrain;
  private double startLeft;
  private double startRight;
  private double setpoint;

  /**
   * Drive the robot a given number of meters with PID control
   * @param drivetrain subsystem
   * @param setpoint in meters
   */
  public PIDdrive(Drivetrain drivetrain, double setpoint) {
    this.drivetrain = drivetrain;
    this.setpoint = setpoint;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    //TODO: test and tune these values, currently arbitrary
    drivetrain.ResetEncoders();
    double p = SmartDashboard.getNumber("pid/drive/p", 0.5);
    double i = SmartDashboard.getNumber("pid/drive/i", 0);
    double d = SmartDashboard.getNumber("pid/drive/d", 0);
    controller = new PIDController(p, i, d);
    controller.setSetpoint(setpoint);
    controller.setTolerance(0.2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double measurement = (drivetrain.getLeftPosition()-startLeft + drivetrain.getRightPosition()-startRight)/2;
    double measurement = (drivetrain.getLeftPosition() + drivetrain.getRightPosition())/2 ;
    double input = controller.calculate(measurement);
    double limit = 0.4;
    input = MathUtil.clamp(input, -limit, limit);
    drivetrain.tankDrive(input, input);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}
