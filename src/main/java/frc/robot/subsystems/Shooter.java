// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SparkMaxSim;
import frc.robot.Constants.ShooterMotorPorts;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final CANSparkMax shooterMotor = new SparkMaxSim(ShooterMotorPorts.shooterWheel, MotorType.kBrushless);
  private final RelativeEncoder shooterEncoder = shooterMotor.getEncoder();

  private Mechanism2d shooterMech = new Mechanism2d(0, 0);

  public Shooter() {
    MechanismRoot2d root = shooterMech.getRoot("shooter", 0, 0);
    MechanismLigament2d shooterBody = new MechanismLigament2d("Shooter Body", 5, 100);
    root.append(shooterBody);

  }

  public void setShooterSpeed(double speed){
    shooterMotor.set(speed);
  }

  public double getShooterPosition(){
    return shooterEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
