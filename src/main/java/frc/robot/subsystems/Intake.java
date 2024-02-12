// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SparkMaxSim;
import frc.robot.Constants.IntakeMechanism;
import frc.robot.Constants.IntakeMotorPorts;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeWheels = new SparkMaxSim(IntakeMotorPorts.kIntakeWheel, MotorType.kBrushless);
  private final CANSparkMax intakePivot = new SparkMaxSim(IntakeMotorPorts.kIntakePivot, MotorType.kBrushless);
  
  private final RelativeEncoder wheelEncoder = intakeWheels.getEncoder();
  private final RelativeEncoder pivotEncoder = intakePivot.getEncoder();

  //SIMULATION
    
  // private final Encoder sEncoderWheels = new Encoder(IntakeMotorPorts.kIntakeWheel, IntakeMotorPorts.kIntakeWheel);
  // private final Encoder sEncoderPivot = new Encoder(IntakeMotorPorts.kIntakePivot, IntakeMotorPorts.kIntakePivot);

  // private final EncoderSim simEncoderL = new EncoderSim(sEncoderWheels);
  // private final EncoderSim simEncoderR = new EncoderSim(sEncoderPivot);

  Mechanism2d intakeArm;
  

  public Intake() {

  }

  public void setIntakeSpeed(double speed){
    intakeWheels.set(speed);
  }

  public void setPivotSpeed(double speed){
    if ((getPivotAngle() == 180 && speed > 0) || (getPivotAngle() == 0 && speed < 0)){
      return;
    }
    intakePivot.set(speed);
  }

  //TODO: experiment with this 
  //TODO: add programmed limit
  // public void setPivotAngle(double angle){

  // }

  public double getWheelPosition(){
    return wheelEncoder.getPosition();
  }

  public double getPivotPosition(){
    return pivotEncoder.getPosition();
  }

  public double getPivotAngle(){
    //TODO: experiment with this later
    pivotEncoder.setPositionConversionFactor(360);
    // return pivotEncoder.getCountsPerRevolution() * 360 * pivotEncoder.getPosition() * IntakeMechanism.pivotGearRatio;
    return pivotEncoder.getPositionConversionFactor() * pivotEncoder.getPosition() * IntakeMechanism.pivotGearRatio;
  }

  public void resetEncoders(){
    wheelEncoder.setPosition(0);
    pivotEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
