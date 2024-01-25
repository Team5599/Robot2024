// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.Drivetrain;

public final class Autos {
  private Autos() {
    // throw new UnsupportedOperationException("This is a utility class!");
  }

  public static Command Test(){
    return new InstantCommand( ()->{
      System.out.println("yes");
    });
  }

  public static Command Leave(Drivetrain drivetrain,double input){
    return new TankDrive(drivetrain, 1).withTimeout(3);
  }
}
