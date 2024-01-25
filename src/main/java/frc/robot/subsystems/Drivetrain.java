// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.spec.EncodedKeySpec;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.jni.CANSWDLJNI;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private CANSparkMax FLMotor; 
  private CANSparkMax BLMotor; 
  private CANSparkMax FRMotor; 
  private CANSparkMax BRMotor;

  private RelativeEncoder FLEncoder = FLMotor.getEncoder(); 
  private RelativeEncoder FREncoder = FRMotor.getEncoder();
  private RelativeEncoder BLEncoder = BLMotor.getEncoder(); 
  private RelativeEncoder BREncoder = BRMotor.getEncoder();
  
  private DifferentialDrive differentialDrive; 

  public Drivetrain() {
    BLMotor.follow(FLMotor); 
    BRMotor.follow(FRMotor);

    differentialDrive = new DifferentialDrive(FLMotor,FRMotor);
   }
  public void tankDrive(double leftSpeed, double rightSpeed){ 
    differentialDrive.tankDrive(leftSpeed, rightSpeed); 
  }
  public double getLeftInput(){ 
    return FLMotor.get(); 
  }
  public double getRightInput(){ 

    return FRMotor.get();
  }

  public double LeftPosition(){
    return (FLEncoder.getPosition() + BLEncoder.getPosition())/ 2 ; 

  }

  public double RightPosition(){ 
    return (FREncoder.getPosition() + BREncoder.getPosition()) / 2; 
  }

  public void ResetEncoders(){
    FLEncoder.setPosition(0);
    FREncoder.setPosition(0);
    BLEncoder.setPosition(0);
    BREncoder.setPosition(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
