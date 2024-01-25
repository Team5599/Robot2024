// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterMotorPorts;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final CANSparkMax shooterMotor = new CANSparkMax(ShooterMotorPorts.shooterWheel, MotorType.kBrushless);
  private final RelativeEncoder shooterEncoder = shooterMotor.getEncoder();

  public Shooter() {}

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