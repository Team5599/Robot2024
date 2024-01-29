// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class SetIntakeAngle extends Command {
  public static enum Level{
    AMP,
    PASSOVER,
    GROUND
  }

  private Intake intake;
  private Level level;
  private PIDController controller;

  public SetIntakeAngle(Intake intake, Level level) {
    this.intake = intake;
    this.level = level;
    
    addRequirements(intake);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //TODO: tune values
    double p = SmartDashboard.getNumber("pid/setIntakeAngle/p", 1);
    double i = SmartDashboard.getNumber("pid/setIntakeAngle/i", 0);
    double d = SmartDashboard.getNumber("pid/setIntakeAngle/d", 0.5);
    controller = new PIDController(p, i, d);
    controller.setTolerance(0);
    
    if(level == Level.AMP) {
      controller.setSetpoint(80);
    }
    else if(level == Level.GROUND){
      controller.setSetpoint(180);
    }
    else if (level == Level.PASSOVER){
      controller.setSetpoint(0);
    }
  }

  @Override
  public void execute() {
    double calculation = controller.calculate(intake.getPivotAngle());
    calculation = MathUtil.clamp(calculation, -1, 1);
    intake.setPivotSpeed(calculation);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}
