// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.Drivetrain;

public class PIDdrive extends Command {
  private PIDController controller;
  private Drivetrain drivetrain;
  private double setpoint;
  private double limit =1;

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
    drivetrain.ResetEncoders();
    controller = new PIDController(PIDConstants.Drive.p, PIDConstants.Drive.i, PIDConstants.Drive.d);
    controller.setP(SmartDashboard.getNumber("pid/drive/p", PIDConstants.Drive.p));
    controller.setI(SmartDashboard.getNumber("pid/drive/i", PIDConstants.Drive.i));
    controller.setD(SmartDashboard.getNumber("pid/drive/d", PIDConstants.Drive.d));

    controller.setSetpoint(setpoint);
    controller.setTolerance(0.025);//an inch of tolerance
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double measurement = (drivetrain.getLeftPosition() + drivetrain.getRightPosition())/2 ;
    double input = controller.calculate(measurement);
    double min = 0.15;
    if (input > 0){
      input = MathUtil.clamp(input, min, limit);
    }
    else if (input < 0){
      input = MathUtil.clamp(input, -limit, -min);
    }
    drivetrain.tankDrive(input, input);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}
