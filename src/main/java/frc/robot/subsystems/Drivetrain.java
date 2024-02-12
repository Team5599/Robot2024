// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.ADIS16470_IMUSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.SparkMaxSim;
import frc.robot.Constants.DrivetrainMechanism;
import frc.robot.Constants.DrivetrainMotorPorts;
import frc.robot.Robot.LimelightHelpers;

public class Drivetrain extends SubsystemBase {
  private CANSparkMax FLMotor = new SparkMaxSim(DrivetrainMotorPorts.kFLmotor, MotorType.kBrushless); 
  private CANSparkMax BLMotor = new SparkMaxSim(DrivetrainMotorPorts.kBLmotor, MotorType.kBrushless); 
  private CANSparkMax FRMotor = new SparkMaxSim(DrivetrainMotorPorts.kFRmotor, MotorType.kBrushless);
  private CANSparkMax BRMotor = new SparkMaxSim(DrivetrainMotorPorts.kBRmotor, MotorType.kBrushless);

  private RelativeEncoder FLEncoder = FLMotor.getEncoder(); 
  private RelativeEncoder FREncoder = FRMotor.getEncoder();
  private RelativeEncoder BLEncoder = BLMotor.getEncoder(); 
  private RelativeEncoder BREncoder = BRMotor.getEncoder();
  
  private DifferentialDrive differentialDrive; 
  private DifferentialDrivePoseEstimator poseEstimator;
  private ADIS16470_IMU imu = new ADIS16470_IMU();

  //SIMULATION

  private final Encoder sEncoderL = new Encoder(DrivetrainMotorPorts.kFLmotor, DrivetrainMotorPorts.kFRmotor);
  private final Encoder sEncoderR = new Encoder(DrivetrainMotorPorts.kBLmotor, DrivetrainMotorPorts.kBRmotor);

  private final EncoderSim simEncoderL = new EncoderSim(sEncoderL);
  private final EncoderSim simEncoderR = new EncoderSim(sEncoderR);

  private final ADIS16470_IMUSim imuSim = new ADIS16470_IMUSim(imu);

  private DifferentialDrivetrainSim SimDrivetrain;

  private Field2d field = new Field2d();


  public Drivetrain() {
    BLMotor.setInverted(true);
    BLMotor.follow(FLMotor); 
    BRMotor.follow(FRMotor);

    SimDrivetrain = new DifferentialDrivetrainSim(
      DCMotor.getNEO(DrivetrainMechanism.kGearboxMotorCount),
      DrivetrainMechanism.kGearBoxRatio,
      DrivetrainMechanism.kCenterMomentInertia,
      DrivetrainMechanism.kDriveBaseWeight,
      Units.inchesToMeters(DrivetrainMechanism.kWheelDiameter / 2),
      Units.inchesToMeters(DrivetrainMechanism.kWheelTrackWidth),
      VecBuilder.fill(0.001, 0.001, 0.001, 0.05, 0.05, 0.005, 0.005)
    );

    differentialDrive = new DifferentialDrive(FLMotor,FRMotor);
    poseEstimator = new DifferentialDrivePoseEstimator(
      new DifferentialDriveKinematics(Units.inchesToMeters(DrivetrainMechanism.kWheelTrackWidth)),
      getRotation(), 
      getLeftPosition(), 
      getRightPosition(),
      // LimelightHelpers.getBotPose2d("limelight")
      new Pose2d()
    );
    poseEstimator.addVisionMeasurement(LimelightHelpers.getBotPose2d(getName()), getLeftInput());

    SmartDashboard.putData("Field", field);
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
    if(Robot.isSimulation()){
      return simEncoderL.getDistance();
    }
    return (FLEncoder.getPosition() + BLEncoder.getPosition()) / 2;
  }

  public double getRightPosition(){ 
    if(Robot.isSimulation()){
      return simEncoderR.getDistance();
    }
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

  public Pose2d getPose2d(){
    return poseEstimator.getEstimatedPosition();
  }

  public void zeroIMU(){
    imu.reset();
    imu.calibrate();
  }
  
  @Override
  public void periodic() {
    poseEstimator.update(
      getRotation(),
      getLeftPosition(),
      getRightPosition()
    );
  }

  @Override
  public void simulationPeriodic(){

    SimDrivetrain.setInputs(getLeftInput() * RobotController.getBatteryVoltage(), getRightInput() * RobotController.getInputVoltage()); 
    // Advance the model by 20 ms.
    SimDrivetrain.update(.02);

    simEncoderL.setDistance(SimDrivetrain.getLeftPositionMeters());
    simEncoderL.setRate(SimDrivetrain.getLeftVelocityMetersPerSecond());
    simEncoderR.setDistance(SimDrivetrain.getRightPositionMeters());
    simEncoderR.setRate(SimDrivetrain.getRightVelocityMetersPerSecond());
    
    imuSim.setGyroAngleX(MathUtil.inputModulus(SimDrivetrain.getHeading().getDegrees(), -180, 180));
    
    field.setRobotPose(SimDrivetrain.getPose());
  }
}
