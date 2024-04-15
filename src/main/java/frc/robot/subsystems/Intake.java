// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.Unit;
import com.revrobotics.Rev2mDistanceSensor;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.SparkMaxSim;
import frc.robot.Constants.IntakeMechanism;
import frc.robot.Constants.IntakePorts;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeWheels = new SparkMaxSim(IntakePorts.kIntakeWheel, MotorType.kBrushless);
  private final CANSparkMax intakePivot = new SparkMaxSim(IntakePorts.kIntakePivot, MotorType.kBrushless);
  
  private final RelativeEncoder wheelEncoder = intakeWheels.getEncoder();
  private final RelativeEncoder pivotEncoder = intakePivot.getEncoder();

  private Rev2mDistanceSensor noteDetector = new Rev2mDistanceSensor(Port.kOnboard);
  //SIMULATION

  private SingleJointedArmSim intakeArmSim;

  private Mechanism2d intakeMech;
  private MechanismLigament2d intakeAntebrachial;
  private double mechScaleFactor = 1;


  public Intake() {
    intakeWheels.setInverted(true);
    pivotEncoder.setPositionConversionFactor(IntakeMechanism.kPositionConversionFactor);
    pivotEncoder.setVelocityConversionFactor(IntakeMechanism.kPositionConversionFactor / 60);

    resetEncoders();//INTAKE MUST START IN THE CURLED POSITION

    if (Robot.isSimulation()){
      configureSimulation();
    }
    noteDetector.setEnabled(true);
    noteDetector.setAutomaticMode(true);
  }

  public void configureSimulation(){
    //SIMULATION
    intakeArmSim = new SingleJointedArmSim(
      IntakeMechanism.intakePlant,
      IntakeMechanism.intakePivotGearBox,
      IntakeMechanism.pivotGearRatio,
      IntakeMechanism.intakeAntebrachialLength,
      Units.degreesToRadians(-10),
      Units.degreesToRadians(180),
      true,
      Units.degreesToRadians(0)
    );

    //MECHANISM
    intakeMech = new Mechanism2d(25, 25);
    MechanismRoot2d root = intakeMech.getRoot("Intake", 5, 1);

    MechanismLigament2d intakeMount = new MechanismLigament2d(
      "Intake Mount",
      Units.metersToInches(IntakeMechanism.intakeMountLength) * mechScaleFactor,
      90.0
    );
    intakeMount.setColor(new Color8Bit(255,0,255));

    intakeAntebrachial = new MechanismLigament2d(
      "Intake Antebrachial",
      Units.metersToInches(IntakeMechanism.intakeAntebrachialLength) * mechScaleFactor,
      -90);
    intakeAntebrachial.setColor(new Color8Bit(0,255,255));

    MechanismLigament2d intakeCarpal = new MechanismLigament2d(
      "Intake Carpal",
      Units.metersToInches(IntakeMechanism.intakeCarpalLength) * mechScaleFactor,
      (IntakeMechanism.intakeWristangle)
    );

    root.append(intakeMount);
    intakeMount.append(intakeAntebrachial);
    intakeAntebrachial.append(intakeCarpal);

    SmartDashboard.putData("intake mechanism", intakeMech);
    SmartDashboard.putNumber("intake/Intake Sim voltage multiplier", 12);
  }

  // speed = MathUtil.clamp(speed, -0.3, 0.3);
  public void setIntakeSpeed(double speed){
    speed *= 0.2;
    intakeWheels.set(speed);
  }

  //TODO: add position limiters
  public void setPivotSpeed(double speed){
    if (Robot.isSimulation()){
      SmartDashboard.putNumber("intake/Intake Sim input in volts:", speed * SmartDashboard.getNumber("intake/Intake Sim voltage multiplier", 1));
      intakeArmSim.setInputVoltage(speed * SmartDashboard.getNumber("intake/Intake Sim voltage multiplier", 1));
    }

    // if ((getPivotPosition() >= 179 && speed > 0) || (getPivotPosition() <= 1 && speed < 0)){
    //   return;
    // }
    intakePivot.set(speed);
  }

  public double getWheelPosition(){
    return wheelEncoder.getPosition();
  }

  //TODO: make sure this is accurate
  public double getPivotPosition(){
    return pivotEncoder.getPosition();
  }
  
  public void resetPivotPosition(){
    pivotEncoder.setPosition(0);
  }

  public double getPivotVelocity(){
    return pivotEncoder.getVelocity();
  }

  public void resetEncoders(){
    wheelEncoder.setPosition(0);
    pivotEncoder.setPosition(0);
  }

  public boolean noteCollected(){
    return (noteDetector.getRange()) < 2.2;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Distance Sensor/distance", noteDetector.getRange(Unit.kInches));

    SmartDashboard.putNumber("intake/Get Pivot Position", getPivotPosition());
  }

  @Override
  public void simulationPeriodic(){
    intakeArmSim.update(0.02);
    intakeAntebrachial.setAngle(getPivotPosition()-90);//offset from parent angle
    SmartDashboard.putNumber("intake/Intake Sim angle in Rads: ", intakeArmSim.getAngleRads());
    SmartDashboard.putNumber("intake/Intake Sim angle in Deg:", Units.radiansToDegrees(intakeArmSim.getAngleRads()));    
  }
}
