// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
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
  private ArmFeedforward feedforward;

  public SetIntakeAngle(Intake intake, Level level) {
    this.intake = intake;
    this.level = level;
    
    addRequirements(intake);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //TODO: tune values, PID may not be enough 
    double p = SmartDashboard.getNumber("pid/setIntakeAngle/p", 1);
    double i = SmartDashboard.getNumber("pid/setIntakeAngle/i", 0);
    double d = SmartDashboard.getNumber("pid/setIntakeAngle/d", 0.5);
    controller = new PIDController(p, i, d);
    controller.setTolerance(2);

    double kS = SmartDashboard.getNumber("", 0);
    double kG = SmartDashboard.getNumber("", 0);
    double kV = SmartDashboard.getNumber("", 0);
    feedforward = new ArmFeedforward(kS, kG, kV);

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
    double PIDCalc = controller.calculate(intake.getPivotPosition());
    double FFCalc = feedforward.calculate(Units.degreesToRadians(intake.getPivotPosition()), Units.degreesToRadians(intake.getPivotVelocity()));
    PIDCalc = MathUtil.clamp(PIDCalc, -0.3, 0.3);
    intake.setPivotSpeed(PIDCalc);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}
