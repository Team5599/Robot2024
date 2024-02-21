// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.SparkMaxSim;
import frc.robot.Constants.ShooterMechanism;
import frc.robot.Constants.ShooterMotorPorts;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final CANSparkMax upperMotor = new SparkMaxSim(ShooterMotorPorts.upperShooterWheels, MotorType.kBrushless);
  private final CANSparkMax lowerMotor = new SparkMaxSim(ShooterMotorPorts.lowerShooterWheels, MotorType.kBrushless);
  private final RelativeEncoder upperEncoder = upperMotor.getEncoder();
  private final RelativeEncoder lowerEncoder = upperMotor.getEncoder();

  //SIM ENCODERS

  private Mechanism2d flywheelMech = new Mechanism2d(20, 20);
  MechanismLigament2d upperWheelSim;
  MechanismLigament2d lowerWheelSim;

  private FlywheelSim upperFlywheelSim;
  private FlywheelSim lowerFlywheelSim;

  public Shooter() {
    lowerMotor.setInverted(true);
    upperEncoder.setPositionConversionFactor(ShooterMechanism.kPositionConversionFactor);
    lowerEncoder.setPositionConversionFactor(ShooterMechanism.kPositionConversionFactor);
    upperEncoder.setPositionConversionFactor(ShooterMechanism.kVelocityConversionFactor);
    lowerEncoder.setPositionConversionFactor(ShooterMechanism.kVelocityConversionFactor);

    configureSimulation();
  }

  public void configureSimulation(){
      upperFlywheelSim = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.0003, 1),
      DCMotor.getNEO(1),
      1
    );

    lowerFlywheelSim = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.0003, 1),
      DCMotor.getNEO(1),
      1
    );

    MechanismRoot2d wheelRoot = flywheelMech.getRoot("flywheelMech", 10, 5);
    upperWheelSim = new MechanismLigament2d("upper wheel",5, -90);
    upperWheelSim.setLineWeight(3);

    MechanismLigament2d wheelLigament = new MechanismLigament2d("wheel ligament", 10, 90);

    lowerWheelSim = new MechanismLigament2d("lower wheel", 5, 0);
    lowerWheelSim.setLineWeight(3);
    lowerWheelSim.setColor(new Color8Bit(0,255,255));

    wheelRoot.append(wheelLigament);
    wheelLigament.append(upperWheelSim);
    wheelRoot.append(lowerWheelSim);
    SmartDashboard.putData("flywheelMech", flywheelMech);
  }

  public void setShooterSpeed(double speed){
    upperFlywheelSim.setInput(speed * 12);
    lowerFlywheelSim.setInput(-speed * 12);
    upperMotor.set(speed);
    lowerMotor.set(speed);
  }

  public double getUpperPosition(){
    return upperEncoder.getPosition();
  }

  public double getLowerPosition(){
    return lowerEncoder.getPosition();
  }

  public double getUpperVelocity(){
    if (Robot.isSimulation()){
      return upperFlywheelSim.getAngularVelocityRPM();
    }
    return upperEncoder.getVelocity();
  }

  public double getLowerVelocity(){
    if (Robot.isSimulation()){
      return lowerFlywheelSim.getAngularVelocityRPM();
    }
    return lowerEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/Upper Velocity", getUpperVelocity());
    SmartDashboard.putNumber("Shooter/Lower Velocity", getLowerVelocity());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic(){
    upperFlywheelSim.update(0.02);
    lowerFlywheelSim.update(0.02);
    upperWheelSim.setAngle((360 * getUpperVelocity() / 60) / 5 + upperWheelSim.getAngle());
    lowerWheelSim.setAngle((360 * getLowerVelocity() / 60)/ 5 + lowerWheelSim.getAngle());
  }
}
