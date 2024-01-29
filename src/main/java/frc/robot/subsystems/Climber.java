// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberMotorPorts;

public class Climber extends SubsystemBase {
  private Spark leftClimber = new Spark(ClimberMotorPorts.leftClimberMotor);
  private Spark rightClimber = new Spark(ClimberMotorPorts.rightClimberMotor);

  public Climber() {
    //TODO: one of these motors may need to be inverted
    // leftClimber.setInverted(true);
  }

  public void activateClimber(double left, double right){
    leftClimber.set(left);
    rightClimber.set(right);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
