// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;

public class TankDrive extends Command {

  private Drivetrain drivetrain;
  private CommandXboxController controller;
  private double input = 0;
  /** Creates a new TankDrive. */
  public TankDrive(Drivetrain drivetrain, CommandXboxController controller) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

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
      leftSpeed = controller.getLeftY();
      rightSpeed = controller.getRightY();
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
