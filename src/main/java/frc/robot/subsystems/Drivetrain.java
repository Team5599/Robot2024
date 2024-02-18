// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.WheelPositions;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.simulation.ADIS16470_IMUSim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
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
  private Pose2d initPose = new Pose2d(0.68, 6.83, new Rotation2d());

  public Drivetrain() {
    FLMotor.setInverted(true);
    BLMotor.follow(FLMotor); 
    BRMotor.follow(FRMotor);
    imu.calibrate();

    FLEncoder.setPositionConversionFactor(DrivetrainMechanism.kPositionConversionFactor);
    FREncoder.setPositionConversionFactor(DrivetrainMechanism.kPositionConversionFactor);
    BLEncoder.setPositionConversionFactor(DrivetrainMechanism.kPositionConversionFactor);
    BREncoder.setPositionConversionFactor(DrivetrainMechanism.kPositionConversionFactor);

    FLEncoder.setVelocityConversionFactor(DrivetrainMechanism.kVelocityConversionFactor);
    FREncoder.setVelocityConversionFactor(DrivetrainMechanism.kVelocityConversionFactor);
    BLEncoder.setVelocityConversionFactor(DrivetrainMechanism.kVelocityConversionFactor);
    BREncoder.setVelocityConversionFactor(DrivetrainMechanism.kVelocityConversionFactor);

    // FLEncoder.setPositionConversionFactor(1);
    // FREncoder.setPositionConversionFactor(1);
    // BLEncoder.setPositionConversionFactor(1);
    // BREncoder.setPositionConversionFactor(1);

    // FLEncoder.setVelocityConversionFactor(1);
    // FREncoder.setVelocityConversionFactor(1);
    // BLEncoder.setVelocityConversionFactor(1);
    // BREncoder.setVelocityConversionFactor(1);

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
      // LimelightHelpers.getBotPose2d("limelight") //TODO: add vision when robot is ready
      new Pose2d()
    );
    // poseEstimator.addVisionMeasurement(LimelightHelpers.getBotPose2d(getName()), getLeftInput());
    configurePathPlanner();

    SimDrivetrain.setPose(initPose);
    field.setRobotPose(initPose);
    
    SmartDashboard.putNumber("Ramsete/b", DrivetrainMechanism.ramseteBeta);
    SmartDashboard.putNumber("Ramsete/z", DrivetrainMechanism.ramseteZeta);

    SmartDashboard.putNumber("PathPlanner/left Input", 0);
    SmartDashboard.putNumber("PathPlanner/right Input", 0);
    SmartDashboard.putNumber("PathPlanner/left wheel speed", 0);
    SmartDashboard.putNumber("PathPlanner/right wheel speed", 0);
    SmartDashboard.putNumber("PathPlanner/Chassis x", 0);
    SmartDashboard.putNumber("PathPlanner/Chassis w", 0);
    SmartDashboard.putData("Field", field);
  }

  public void configurePathPlanner(){
    AutoBuilder.configureRamsete(
      this::getPose, 
      this::resetPose,
      this::getChassisSpeeds,
      this::driveChassisSpeed, 
      SmartDashboard.getNumber("Ramsete/b", 1.25),
      SmartDashboard.getNumber("Ramsete/z", 0.7),
      new ReplanningConfig(),
      ()->{
        // if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
        //   return true;
        // }
        return false;
      }, 
      this
    );
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

  public double getLeftVelocity(){
    if(Robot.isSimulation()){
      return simEncoderL.getRate();
    }
    return FLEncoder.getVelocity();
  }

  public double getRightPosition(){ 
    if(Robot.isSimulation()){
      return simEncoderR.getDistance();
    }
    return (FREncoder.getPosition() + BREncoder.getPosition()) / 2; 
  }

  public double getRightVelocity(){
    if(Robot.isSimulation()){
      return simEncoderR.getRate();
    }
    return FREncoder.getVelocity();
  }

  public Rotation2d getRotation(){
    return Rotation2d.fromDegrees(imu.getAngle(ADIS16470_IMU.IMUAxis.kZ));
  }

  public void ResetEncoders(){
    FLEncoder.setPosition(0);
    FREncoder.setPosition(0);
    BLEncoder.setPosition(0);
    BREncoder.setPosition(0);
    
    sEncoderL.reset();
    sEncoderR.reset();
    simEncoderL.setDistance(0);
    simEncoderR.setDistance(0);
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

  public Pose2d getPose(){
    if (Robot.isSimulation()){
      return SimDrivetrain.getPose();
    }
    return poseEstimator.getEstimatedPosition();
  }

  //ADVANCED
  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    if (Robot.isSimulation()){
      return new DifferentialDriveWheelSpeeds(
        SimDrivetrain.getLeftVelocityMetersPerSecond(),
        SimDrivetrain.getRightVelocityMetersPerSecond()
        );
      }

    return new DifferentialDriveWheelSpeeds(
      FLEncoder.getVelocity(),
      FREncoder.getVelocity()
    );
  }

  public ChassisSpeeds getChassisSpeeds(){
    return DrivetrainMechanism.driveKinematics.toChassisSpeeds(getWheelSpeeds());
  }
  
  public void resetPose(Pose2d pose){
    SimDrivetrain.setPose(pose);
    // poseEstimator.resetPosition(getRotation(), null, pose);
  }

  public void resetIMU(){
    imu.reset();
  }

  public void driveChassisSpeed(ChassisSpeeds chassisSpeeds){
    SmartDashboard.putNumber("PathPlanner/Chassis x", chassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("PathPlanner/Chassis w", chassisSpeeds.omegaRadiansPerSecond);
    DifferentialDriveWheelSpeeds wheelSpeeds = DrivetrainMechanism.driveKinematics.toWheelSpeeds(chassisSpeeds);
    SmartDashboard.putNumber("PathPlanner/left wheel speed", wheelSpeeds.leftMetersPerSecond);
    SmartDashboard.putNumber("PathPlanner/right wheel speed", wheelSpeeds.rightMetersPerSecond);

    //TODO: make sure these return a value between 1 and -1
    double scaleFactor = 6;
    if (Robot.isReal()){
      scaleFactor = DrivetrainMechanism.kVelocityConversionFactor;
    }
    double leftInput = wheelSpeeds.leftMetersPerSecond/scaleFactor;
    double rightInput = wheelSpeeds.rightMetersPerSecond/scaleFactor;
    SmartDashboard.putNumber("PathPlanner/left Input", leftInput);
    SmartDashboard.putNumber("PathPlanner/right Input", rightInput);

    tankDrive(leftInput, rightInput);
  }

  public void voltageDrive(Measure<Voltage> voltageMeasure){
    double maxVoltage = 12;
    differentialDrive.tankDrive(voltageMeasure.divide(maxVoltage).baseUnitMagnitude(), voltageMeasure.divide(maxVoltage).baseUnitMagnitude());
  }
  
  @Override
  public void periodic() {
    poseEstimator.update(
      getRotation(),
      getLeftPosition(),
      getRightPosition()
    );

    SmartDashboard.putNumber("Drivetrain/Left Position", getLeftPosition());
    SmartDashboard.putNumber("Drivetrain/Right Position", getRightPosition());

    SmartDashboard.putNumber("Drivetrain/Left velocity", getLeftVelocity());
    SmartDashboard.putNumber("Drivetrain/Right velocity", getRightVelocity());
    

    // SmartDashboard.putNumber("Drivetrain/Angle X", getGyroAngleX());
    // SmartDashboard.putNumber("Drivetrain/Angle Y", getGyroAngleY());
    // SmartDashboard.putNumber("Drivetrain/Angle Z", getGyroAngleZ());

    // SmartDashboard.putNumber("PathPlanner/left wheel speed", getWheelSpeeds().leftMetersPerSecond);
    // SmartDashboard.putNumber("PathPlanner/right wheel speed", getWheelSpeeds().rightMetersPerSecond);

    // SmartDashboard.putNumber("PathPlanner/Chassis x", getChassisSpeeds().vxMetersPerSecond);
    // SmartDashboard.putNumber("PathPlanner/Chassis w", getChassisSpeeds().omegaRadiansPerSecond);

    SmartDashboard.putNumber("Drivetrain/Pose x", getPose().getX());
    SmartDashboard.putNumber("Drivetrain/Pose y", getPose().getY());
    SmartDashboard.putNumber("Drivetrain/Pose r", getPose().getRotation().getDegrees());

  }


  @Override
  public void simulationPeriodic(){
    SimDrivetrain.setInputs(getLeftInput() * RobotController.getBatteryVoltage(), getRightInput() * RobotController.getInputVoltage()); 
    // Advance the model by 20 ms.
    
    simEncoderL.setDistance(SimDrivetrain.getLeftPositionMeters());
    simEncoderL.setRate(SimDrivetrain.getLeftVelocityMetersPerSecond());
    simEncoderR.setDistance(SimDrivetrain.getRightPositionMeters());
    simEncoderR.setRate(SimDrivetrain.getRightVelocityMetersPerSecond());

    SimDrivetrain.update(.02);
    
    imuSim.setGyroAngleX(MathUtil.inputModulus(SimDrivetrain.getHeading().getDegrees(), -180, 180));
    
    field.setRobotPose(SimDrivetrain.getPose());
  }
}
