// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

public class DriveTrain extends SubsystemBase {

  DifferentialDrive driveTrain;

  
  private final CANSparkMax MCBL = new CANSparkMax(OperatorConstants.funnyNumber, MotorType.kBrushless);
  private final CANSparkMax MCBR = new CANSparkMax(OperatorConstants.funnyNumber, MotorType.kBrushless);
  private final CANSparkMax MCFL = new CANSparkMax(OperatorConstants.funnyNumber, MotorType.kBrushless);
  private final CANSparkMax MCFR = new CANSparkMax(OperatorConstants.funnyNumber, MotorType.kBrushless);
  

  private RelativeEncoder EncoderFL = MCFL.getEncoder();
  private RelativeEncoder EncoderBL = MCBL.getEncoder();
  private RelativeEncoder EncoderFR = MCFR.getEncoder();
  private RelativeEncoder EncoderBR = MCBR.getEncoder();
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    MCFL.follow(MCBL);
    MCFR.follow(MCBR);
    driveTrain = new DifferentialDrive(MCFL, MCFR);
      
  }
  public void tankDrive(double leftSpeed, double rightSpeed) {
    driveTrain.tankDrive(leftSpeed, rightSpeed);// totally on my first try
  }
  
  public double getLeftEncoderPos(){
    double FLPosition = EncoderFL.getPosition();
    double BLPosition = EncoderBL.getPosition();
    return (FLPosition + BLPosition) / 2;
  }

  public double getRightEncoderPos(){
    double FRPosition = EncoderFR.getPosition();
    double BRPosition = EncoderBR.getPosition();
    return (FRPosition + BRPosition) / 2;
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
