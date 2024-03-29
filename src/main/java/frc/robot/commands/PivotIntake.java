// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;

public class PivotIntake extends Command {
  private Intake intake;
  private CommandXboxController joystick;

  private SlewRateLimiter rateLimiter = new SlewRateLimiter(0.5);

  public PivotIntake(Intake intake, CommandXboxController joystick) {
    this.intake = intake;
    this.joystick = joystick;
    addRequirements(intake); 
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double speed = joystick.getLeftY();
    if(Robot.isSimulation()){
      speed = joystick.getRawAxis(0);
    }
    else speed *= 0.3;
    // speed = rateLimiter.calculate(speed);
    intake.setPivotSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
