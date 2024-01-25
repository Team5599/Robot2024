// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeMechanism;
import frc.robot.Constants.IntakeMotorPorts;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeWheels = new CANSparkMax(IntakeMotorPorts.kIntakeWheel, MotorType.kBrushless);
  private final CANSparkMax intakePivot = new CANSparkMax(IntakeMotorPorts.kIntakePivot, MotorType.kBrushless);
  
  private final RelativeEncoder wheelEncoder = intakeWheels.getEncoder();
  private final RelativeEncoder pivotEncoder = intakePivot.getEncoder();

  public Intake() {}

  public void setIntakeSpeed(double speed){
    intakeWheels.set(speed);
  }

  public void setPivotSpeed(double speed){
    intakePivot.set(speed);
  }

  //TODO: experiment with this 
  //TODO: add programmed limit
  public void setPivotAngle(double angle){

  }

  public double getWheelPosition(){
    return wheelEncoder.getPosition();
  }

  public double getPivotPosition(){
    return pivotEncoder.getPosition();
  }

  public double getPivotAngle(){
    //TODO: experiment with this later
    return pivotEncoder.getPositionConversionFactor() * pivotEncoder.getPosition() * IntakeMechanism.pivotGearRatio;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
