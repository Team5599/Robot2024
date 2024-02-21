// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;


import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Pipelines;
import frc.robot.Robot.LimelightHelpers;
import frc.robot.commands.ActivateIntake;
import frc.robot.commands.ActivateShooter;
import frc.robot.commands.TankDrive;
import frc.robot.commands.Auto.SetIntakeAngle.Level;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

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
    return new PathPlannerAuto("Testing Commands");
    // return new PathPlannerAuto("Test 1");
    // return new PathPlannerAuto("S1 test");
  }
  
  // TODO: create a proper autonomous routine
  public static Command Score(Drivetrain drivetrain, Shooter shooter){
    return new SequentialCommandGroup(
      new FaceLLTarget(drivetrain),
      new PIDturn(drivetrain, 180),
      new ActivateShooter(shooter).withTimeout(0.25),
      new PIDdrive(drivetrain, -2),
      new PIDturn(drivetrain, 180)
    );
  }

  //TELEOP AUTOMATION
  public static Command CollectNote(Intake intake){
    return new ActivateIntake(intake)
    .until(
      () -> {return intake.noteCollected();}
    )
    .withTimeout(2)
    .andThen(
      new SetIntakeAngle(intake, Level.PASSOVER)
    );
  }

  public static Command FaceSpeaker(Drivetrain drivetrain){
    if (DriverStation.getAlliance().get() == Alliance.Blue){
      LimelightHelpers.setPipelineIndex("", Pipelines.BLUE.SPEAKER);
    }
    else if (DriverStation.getAlliance().get() == Alliance.Red){
      LimelightHelpers.setPipelineIndex("", Pipelines.RED.SPEAKER);
    }
    return new FaceLLTarget(drivetrain);
  }

}
