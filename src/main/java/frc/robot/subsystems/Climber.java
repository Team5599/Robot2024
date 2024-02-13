// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Robot;
import frc.robot.SparkMaxSim;
import frc.robot.Constants.ClimberMechanism;
import frc.robot.Constants.ClimberMotorPorts;

public class Climber extends SubsystemBase {
  private Spark leftClimber = new Spark(ClimberMotorPorts.leftClimberMotor);
  private Spark rightClimber = new Spark(ClimberMotorPorts.rightClimberMotor);

  private Mechanism2d climberMech = new Mechanism2d(60, 60);
  private MechanismLigament2d climberLigament;
  private double mechScaleFactor = 0.25;

  private ElevatorSim climberSim;
  public Climber() {
    //TODO: one of these motors may need to be inverted
    // leftClimber.setInverted(true);

    //SIMULATION
    MechanismRoot2d root = climberMech.getRoot("Climber Root", 3, 0);
    climberLigament = new MechanismLigament2d("Climber", ClimberMechanism.contractedLength * mechScaleFactor, 90);

    root.append(climberLigament);
    climberSim = new ElevatorSim(
      ClimberMechanism.climberPlant,
      DCMotor.getAndymarkRs775_125(1),
      ClimberMechanism.contractedLength,
      ClimberMechanism.contractedLength + ClimberMechanism.extendedLength,
      false,
      ClimberMechanism.contractedLength + 0.01
    );

    SmartDashboard.putData("Climber Mechanism", climberMech);
    SmartDashboard.putNumber("climber/Climber Sim voltage multiplier", 12);
  }

  public void activateClimber(double left, double right){
    SmartDashboard.putNumber("climber/Climber left input", left);
    SmartDashboard.putNumber("climber/Climber right `input", right);

    if (Robot.isSimulation()){
      SmartDashboard.putNumber("climber/Climber Sim input in volts:", left * SmartDashboard.getNumber("climber/Climber Sim voltage multiplier", 1));
      climberSim.setInputVoltage(left * SmartDashboard.getNumber("climber/Climber Sim voltage multiplier", 1));
    }
    leftClimber.set(left);
    rightClimber.set(right);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override 
  public void simulationPeriodic(){
    climberSim.update(0.02);
    climberLigament.setLength(Units.metersToInches(climberSim.getPositionMeters()));
    // SmartDashboard.putNumber("climber/Climber left input", get)
    SmartDashboard.putNumber("climber/Climber Sim position inches", Units.metersToInches(climberSim.getPositionMeters()));
  }
}
