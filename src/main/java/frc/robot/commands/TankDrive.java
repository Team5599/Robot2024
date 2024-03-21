// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;

public class TankDrive extends Command {

  private Drivetrain drivetrain;
  private CommandPS5Controller controller;
  // private CommandXboxController controller;
  private double input = 0;//input is used here so that a double can be set from the outside, instead of using a double supplier

  private SlewRateLimiter leftLimiter = new SlewRateLimiter(0.5);
  private SlewRateLimiter rightLimiter = new SlewRateLimiter(0.5);

  public TankDrive(Drivetrain drivetrain, CommandPS5Controller controller) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    addRequirements(drivetrain);
  }

  // public TankDrive(Drivetrain drivetrain, CommandXboxController controller) {
  //   this.drivetrain = drivetrain;
  //   this.controller = controller;
  //   addRequirements(drivetrain);
  // }

  public TankDrive(Drivetrain drivetrain, double input){
    this.drivetrain = drivetrain;
    input = MathUtil.clamp(input, -1, 1);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double leftSpeed = 0;
    double rightSpeed = 0;
    if (controller != null){
      //controller up is a negative value, so invert it 
      leftSpeed = -controller.getLeftY();
      rightSpeed = -controller.getRightY();

      //scale factor
      leftSpeed *= 0.85;
      rightSpeed *= 0.85;

      // leftSpeed = leftLimiter.calculate(leftSpeed);
      // rightSpeed = rightLimiter.calculate(rightSpeed);

    } else {
      leftSpeed = input;
      rightSpeed = input;
    }

    drivetrain.tankDrive(leftSpeed, rightSpeed);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
