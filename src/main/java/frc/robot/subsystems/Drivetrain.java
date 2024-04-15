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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.ADIS16470_IMUSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.SparkMaxSim;
import frc.robot.Constants.DrivetrainMechanism;
import frc.robot.Constants.DrivetrainMotorPorts;

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

  private PIDController leftController = new PIDController(2, 0, 0);
  private PIDController rightController = new PIDController(2, 0, 0);

  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DrivetrainMechanism.kS, DrivetrainMechanism.kV, DrivetrainMechanism.kA);;

  //SIMULATION

  private final ADIS16470_IMUSim imuSim = new ADIS16470_IMUSim(imu);

  private DifferentialDrivetrainSim SimDrivetrain;

  private Field2d field = new Field2d();
  private FieldObject2d note1;
  private Pose2d initPose = new Pose2d(0.6, 6.45, new Rotation2d(Units.degreesToRadians(52.82)));

  public Drivetrain() {
    FLMotor.setInverted(false);
    FRMotor.setInverted(true);

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

    configureSimulation();
    configurePathPlanner();
    
    SmartDashboard.putNumber("PathPlanner/left Input", 0);
    SmartDashboard.putNumber("PathPlanner/right Input", 0);
    SmartDashboard.putNumber("PathPlanner/left wheel speed", 0);
    SmartDashboard.putNumber("PathPlanner/right wheel speed", 0);
    SmartDashboard.putNumber("PathPlanner/Chassis x", 0);
    SmartDashboard.putNumber("PathPlanner/Chassis w", 0);
    SmartDashboard.putData("Field", field);
  }

  public void configureSimulation(){
    SimDrivetrain = new DifferentialDrivetrainSim(
      DCMotor.getNEO(DrivetrainMechanism.kGearboxMotorCount),
      DrivetrainMechanism.kGearBoxRatio,
      DrivetrainMechanism.kCenterMomentInertia,
      DrivetrainMechanism.kDriveBaseWeight,
      Units.inchesToMeters(DrivetrainMechanism.kWheelDiameter / 2),
      Units.inchesToMeters(DrivetrainMechanism.kWheelTrackWidth),
      VecBuilder.fill(0.000, 0.000, 0.000, 0.00, 0.00, 0.000, 0.0001)
    );

    differentialDrive = new DifferentialDrive(FLMotor,FRMotor);

    poseEstimator = new DifferentialDrivePoseEstimator(
      new DifferentialDriveKinematics(Units.inchesToMeters(DrivetrainMechanism.kWheelTrackWidth)),
      getRotation(), 
      getLeftPosition(), 
      getRightPosition(),
      new Pose2d()
      );

    if (Robot.isSimulation()){
      SimDrivetrain.setPose(initPose);
      field.setRobotPose(initPose);
      note1 = field.getObject("note1");
      note1.setPose(new Pose2d());
    }
  }

  public void configurePathPlanner(){
    AutoBuilder.configureRamsete(
      this::getPose, 
      this::resetPose,
      this::getChassisSpeeds,
      this::driveChassisSpeed, 
      DrivetrainMechanism.ramseteBeta,
      DrivetrainMechanism.ramseteZeta,
      new ReplanningConfig(),
      ()->{
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
          return true;
        }
        return false;
      }, 
      this
    );
  }

  public void tankDrive(double leftSpeed, double rightSpeed){
    //positive inputs should move the robot forward
    differentialDrive.tankDrive(leftSpeed, rightSpeed); 
  }

  public double getLeftInput(){ 
    return FLMotor.get(); 
  }

  public double getRightInput(){ 
    return FRMotor.get();
  }

  public double getLeftVoltage(){
    return FLMotor.getBusVoltage();
  }

  public double getRightVoltage(){
    return FRMotor.getBusVoltage();
  }

  public double getAverageVoltage(){
    return (getLeftVoltage() + getRightVoltage())/2; 
  }

  public double getLeftPosition(){
    return (FLEncoder.getPosition() + BLEncoder.getPosition()) / 2;
  }

  public double getLeftVelocity(){
    return FLEncoder.getVelocity();
  }

  public double getRightPosition(){ 
    return (FREncoder.getPosition() + BREncoder.getPosition()) / 2; 
  }

  public double getRightVelocity(){
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
    // SimDrivetrain.setPose(pose);
    poseEstimator.resetPosition(getRotation(), null, pose);
  }

  public void resetIMU(){
    imu.reset();
  }

  public void driveChassisSpeed(ChassisSpeeds chassisSpeeds){
    DifferentialDriveWheelSpeeds wheelSpeeds = DrivetrainMechanism.driveKinematics.toWheelSpeeds(chassisSpeeds);

    double leftFeedforward = feedforward.calculate(wheelSpeeds.leftMetersPerSecond);
    double rightFeedforward = feedforward.calculate(wheelSpeeds.rightMetersPerSecond);

    double leftPID = leftController.calculate(getLeftVelocity(), wheelSpeeds.leftMetersPerSecond);
    double rightPID = rightController.calculate(getRightVelocity(), wheelSpeeds.rightMetersPerSecond);

    tankDrive(leftFeedforward + leftPID, rightFeedforward + rightPID);

    // double scaleFactor = 5.5;
    // double leftInput = wheelSpeeds.leftMetersPerSecond/scaleFactor;
    // double rightInput = wheelSpeeds.rightMetersPerSecond/scaleFactor;

    // tankDrive(leftInput, rightInput);
  }

  public void voltageDrive(Measure<Voltage> voltageMeasure){
    double maxVoltage = 12;
    differentialDrive.tankDrive(voltageMeasure.divide(maxVoltage).baseUnitMagnitude(), voltageMeasure.divide(maxVoltage).baseUnitMagnitude());
    SmartDashboard.putNumber("Drivetrain/SysID Voltage", voltageMeasure.divide(maxVoltage).baseUnitMagnitude());
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

    SmartDashboard.putNumber("Drivetrain/Chassis X", getChassisSpeeds().vxMetersPerSecond);
    SmartDashboard.putNumber("Drivetrain/Chassis W", getChassisSpeeds().omegaRadiansPerSecond);    
    
    
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

    SmartDashboard.putNumber("Drivetrain/Left input", getLeftInput());
    SmartDashboard.putNumber("Drivetrain/Right input", getRightInput());

    SmartDashboard.putNumber("Drivetrain/Left Voltage", getLeftVoltage());
    SmartDashboard.putNumber("Drivetrain/Right Voltage", getRightVoltage());
    SmartDashboard.putNumber("Drivetrain/AVerage Voltage", getAverageVoltage());
  }


  @Override
  public void simulationPeriodic(){
    SimDrivetrain.setInputs(getLeftInput() * RobotController.getBatteryVoltage(), getRightInput() * RobotController.getInputVoltage()); 
    // Advance the model by 20 ms.
    
    // simEncoderL.setDistance(SimDrivetrain.getLeftPositionMeters());
    FREncoder.setPosition(SimDrivetrain.getLeftPositionMeters());
    FREncoder.setPosition(SimDrivetrain.getLeftPositionMeters());
    BLEncoder.setPosition(SimDrivetrain.getRightPositionMeters());
    BREncoder.setPosition(SimDrivetrain.getRightPositionMeters());
    
    // simEncoderL.setRate(SimDrivetrain.getLeftVelocityMetersPerSecond());
    // simEncoderR.setDistance(SimDrivetrain.getRightPositionMeters());
    // simEncoderR.setRate(SimDrivetrain.getRightVelocityMetersPerSecond());

    SimDrivetrain.update(.02);
    
    imuSim.setGyroAngleX(MathUtil.inputModulus(SimDrivetrain.getHeading().getDegrees(), -180, 180));
    
    field.setRobotPose(SimDrivetrain.getPose());
  }
}
