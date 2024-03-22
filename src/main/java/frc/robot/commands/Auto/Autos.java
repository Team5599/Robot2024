// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
    throw new UnsupportedOperationException("This is a utility class!");
  }

  //for now, this shoots and then drives forward
  public static Command AutoTest(Drivetrain drivetrain, Intake intake, Shooter shooter){
    Command timedShot = ((new ActivateShooter(shooter)).alongWith(new ActivateIntake(intake, -0.3))).withTimeout(0.2);

    return new SequentialCommandGroup(
      timedShot,
      new PIDdrive(drivetrain, 2.2)
    );
  }
  
  //the "naive" auto just uses basic straight line paths 
  public static Command NaiveAuto(Drivetrain drivetrain, Intake intake, Shooter shooter, boolean isBlue){
    Command timedShot = ((new ActivateShooter(shooter)).alongWith(new ActivateIntake(intake, -0.3))).withTimeout(0.2);
    return new SequentialCommandGroup(
      timedShot,
      new SetIntakeAngle(intake, Level.GROUND),
      new PIDdrive(drivetrain, 2.2).deadlineWith(new ActivateIntake(intake, 0.2)),
      new PIDdrive(drivetrain, -2.2),
      (new ActivateShooter(shooter)).alongWith(new ActivateIntake(intake, -0.3)).withTimeout(0.2)
    );


    // TODO: seems to be a source of error, getting the alliance causes errors when not connected to the FMS
    // try{
    //   if (DriverStation.getAlliance().get() == Alliance.Blue){
    //     LimelightHelpers.setPipelineIndex("", Pipelines.BLUE.SPEAKER);
    //     return new SequentialCommandGroup(
    //       new ActivateShooter(shooter).withTimeout(0.1).alongWith(new ActivateIntake(intake, -0.3).withTimeout(0.1)),
    //       new PIDdrive(drivetrain, 2).deadlineWith(new SetIntakeAngle(intake, Level.GROUND).andThen(new ActivateIntake(intake, 0.2))),
    //       new PIDdrive(drivetrain, -2),
    //       new ActivateShooter(shooter).withTimeout(0.1).alongWith(new ActivateIntake(intake, -0.3).withTimeout(0.1))//Gets us 2 points from this

    //     );
    //   }
    //   else if (DriverStation.getAlliance().get() == Alliance.Red){
    //     LimelightHelpers.setPipelineIndex("", Pipelines.RED.SPEAKER);
    //     return new SequentialCommandGroup(
    //       new ActivateShooter(shooter).withTimeout(0.1).alongWith(new ActivateIntake(intake, -0.3).withTimeout(0.1)),
    //       new PIDdrive(drivetrain, 2).deadlineWith(new SetIntakeAngle(intake, Level.GROUND).andThen(new ActivateIntake(intake, 0.2))),
    //       new PIDdrive(drivetrain, -2),
    //       new ActivateShooter(shooter).withTimeout(0.1).alongWith(new ActivateIntake(intake, -0.3).withTimeout(0.1))//Gets us 2 points from this
    //     );
    //   }
    //   else return new SequentialCommandGroup(        
    //       new ActivateShooter(shooter).withTimeout(0.1).alongWith(new ActivateIntake(intake, -0.3).withTimeout(0.1)),
    //       new PIDdrive(drivetrain,2)
    //     );
    // }
    // catch (Exception e){
    //   e.printStackTrace();
    //   if (isBlue == true){
    //     LimelightHelpers.setPipelineIndex("", Pipelines.BLUE.SPEAKER);
    //     return new SequentialCommandGroup(
    //       new ActivateShooter(shooter).withTimeout(0.1).alongWith(new ActivateIntake(intake, -0.3).withTimeout(0.1)),
    //       new PIDdrive(drivetrain, 2).deadlineWith(new SetIntakeAngle(intake, Level.GROUND).andThen(new ActivateIntake(intake, 0.2))),
    //       new PIDdrive(drivetrain, -2),
    //       new ActivateShooter(shooter).withTimeout(0.1).alongWith(new ActivateIntake(intake, -0.3).withTimeout(0.1))//Gets us 2 points from this
    //     );
    //   }
    //   else if (isBlue == false){
    //     LimelightHelpers.setPipelineIndex("", Pipelines.RED.SPEAKER);
    //     return new SequentialCommandGroup(
    //       new ActivateShooter(shooter).withTimeout(0.1).alongWith(new ActivateIntake(intake, -0.3).withTimeout(0.1)),
    //       new PIDdrive(drivetrain, 2).deadlineWith(new SetIntakeAngle(intake, Level.GROUND).andThen(new ActivateIntake(intake, 0.2))),
    //       new PIDdrive(drivetrain, -2),
    //       new ActivateShooter(shooter).withTimeout(0.1).alongWith(new ActivateIntake(intake, -0.3).withTimeout(0.1))//Gets us 2 points from this
    //     );
    //   }
    //   else return new SequentialCommandGroup(        
    //     new ActivateShooter(shooter).withTimeout(0.1).alongWith(new ActivateIntake(intake, -0.3).withTimeout(0.1)),
    //     new PIDdrive(drivetrain,2)
    //   );
    // }
  }

  public static Command sysId(SysIdRoutine.Direction direction, Drivetrain drivetrain){
    SysIdRoutine routine = new SysIdRoutine(
      new SysIdRoutine.Config(), 
      new SysIdRoutine.Mechanism(drivetrain::voltageDrive, null, drivetrain)
    );
    return routine.quasistatic(direction).withTimeout(1);
  }

  public static Command PathPlannerTest(Drivetrain drivetrain){ 
    // return new PathPlannerAuto("Testing Commands");
    // return new PathPlannerAuto("Test 1");
    return new PathPlannerAuto("S1 test");
  }

  public static Command PIDdriveTest(Drivetrain drivetrain, double setPoint){
    return new PIDdrive(drivetrain, setPoint);
  }

  public static Command PIDturn(Drivetrain drivetrain, double setPoint){ 
    return new PIDturn(drivetrain, setPoint);
  }

  public static Command PIDpivot(Intake intake, Level level){
    return new SetIntakeAngle(intake, level);
  }
  
  //TELEOP AUTOMATION
  public static Command CollectNote(Intake intake){
    return new ActivateIntake(intake,0.2)
    .until(
      () -> {return intake.noteCollected();}
    )
    .withTimeout(2)
    .andThen(
      new SetIntakeAngle(intake, Level.PASSOVER)
    );
  }

  public static Trigger ShooterInRange(){
    if (DriverStation.getAlliance().get() == Alliance.Blue){
      LimelightHelpers.setPipelineIndex("", Pipelines.BLUE.SPEAKER);
    }
    else if (DriverStation.getAlliance().get() == Alliance.Red){
      LimelightHelpers.setPipelineIndex("", Pipelines.RED.SPEAKER);
    }

    Trigger inRange = new Trigger(
      ()-> LimelightHelpers.getBotPose("").length < Units.feetToMeters(3.2));
      // .onTrue(new InstantCommand(()->RobotContainer.driver.getHID().setRumble(RumbleType.kBothRumble, 0.2))
    // );
    return inRange;
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
