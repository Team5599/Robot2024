// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LED;

public class LEDstrip extends SubsystemBase {
  private static LEDstrip instance;
  private Spark led = new Spark(10);

  public LEDstrip() {
    if (instance == null){
      instance = this;
    }
    else{
      throw new UnsupportedOperationException("Attempted to instantiate singleton!");
    }
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("LED/", )
  }
}
