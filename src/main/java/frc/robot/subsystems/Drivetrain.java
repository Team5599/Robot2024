// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainMechanism;
import frc.robot.Robot.LimelightHelpers;

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
  private DifferentialDrivePoseEstimator poseEstimator;
  private ADIS16470_IMU imu = new ADIS16470_IMU();

  public Drivetrain() {
    BLMotor.setInverted(true);
    BLMotor.follow(FLMotor); 
    BRMotor.follow(FRMotor);

    differentialDrive = new DifferentialDrive(FLMotor,FRMotor);
    poseEstimator = new DifferentialDrivePoseEstimator(
      new DifferentialDriveKinematics(DrivetrainMechanism.trackWidth),
      getRotation(), 
      getLeftPosition(), 
      getRightPosition(), 
      LimelightHelpers.getBotPose2d("limelight")
    );
    poseEstimator.addVisionMeasurement(LimelightHelpers.getBotPose2d(getName()), getLeftInput());
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

  public double getLeftPosition(){
    return (FLEncoder.getPosition() + BLEncoder.getPosition()) / 2;
  }

  public double getRightPosition(){ 
    return (FREncoder.getPosition() + BREncoder.getPosition()) / 2; 
  }

  public Rotation2d getRotation(){
    return Rotation2d.fromDegrees(imu.getAngle(ADIS16470_IMU.IMUAxis.kZ));
  }

  public void ResetEncoders(){
    FLEncoder.setPosition(0);
    FREncoder.setPosition(0);
    BLEncoder.setPosition(0);
    BREncoder.setPosition(0);
  }

  public double getGyroAngleZ(){
    return imu.getAngle(IMUAxis.kZ);
  }

  public double getGyroAngleX(){
    return imu.getAngle(IMUAxis.kX);
  }

  public double getGyroAngleY(){
    return imu.getAngle(IMUAxis.kY);
  }

  @Override
  public void periodic() {
    // poseEstimator.update(getRotation(), null)
  }
}
