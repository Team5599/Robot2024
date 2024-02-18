// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;


import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot.LimelightHelpers;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.Drivetrain;

public final class Autos {
  private Autos() {
    // throw new UnsupportedOperationException("This is a utility class!");
  }

  public static Command Leave(Drivetrain drivetrain,double input){
    return new TankDrive(drivetrain, 1).withTimeout(3);
  }

  public static Command PIDdriveTest(Drivetrain drivetrain, double setPoint){
    return new PIDdrive(drivetrain, setPoint);
  }

  public static Command sysId(SysIdRoutine.Direction direction, Drivetrain drivetrain){
    SysIdRoutine routine = new SysIdRoutine(
      new SysIdRoutine.Config(), 
      new SysIdRoutine.Mechanism(drivetrain::voltageDrive, null, drivetrain)
    );
    return routine.quasistatic(direction);
  }

  public static Command PathPlannerTest(Drivetrain drivetrain){
    return new PathPlannerAuto("Test 1");
  }
  
  //TODO: create a proper autonomous routine
  // public static Command Score(Drivetrain drivetrain){
  //   return new SequentialCommandGroup(
  //     PIDturn(drivetrain, )
  //   );
  // }
}
