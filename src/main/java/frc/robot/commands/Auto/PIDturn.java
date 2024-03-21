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

public class PIDturn extends Command {
  private Drivetrain drivetrain;
  private PIDController controller;
  private double initialAngle;//inital angle
  private double angle;//number of degrees to turn by, ccw is assumed to be positive

  public PIDturn(Drivetrain drivetrain, double angle) {
    this.drivetrain = drivetrain;
    this.angle = angle;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    initialAngle = drivetrain.getGyroAngleZ();

    double p = PIDConstants.Turn.p;
    double i = PIDConstants.Turn.i;
    double d = PIDConstants.Turn.d;
    controller = new PIDController(p, i, d);
    controller.setSetpoint(angle);
    SmartDashboard.putNumber("pid/turn/SetPoint", initialAngle);
    controller.setTolerance(2);
    drivetrain.ResetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double measurement = differenceInAngle(drivetrain.getGyroAngleZ(), initialAngle);
    double input = controller.calculate(measurement);
    double limit = 1;
    if (input > 0){
      input = MathUtil.clamp(input, 0.15, limit);
    }
    else if (input < 0){
      input = MathUtil.clamp(input, -limit, -0.15);
    }
    drivetrain.tankDrive(-input, input);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }

  private double differenceInAngle(double a, double b){
    if (Math.abs(a-b) < 180){
      return a-b;
    } 
    else if (Math.abs(a-b) > 180){
      return (a-b) - Math.signum(a-b) * 360.0;
    }
    else return 180;//turn counterclockwise for 180 degree turns
  }
}
