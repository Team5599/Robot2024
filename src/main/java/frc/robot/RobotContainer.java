// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerPorts;
import frc.robot.commands.ActivateClimber;
import frc.robot.commands.ActivateIntake;
import frc.robot.commands.ActivateShooter;
import frc.robot.commands.PivotIntake;
import frc.robot.commands.ShootIntake;
import frc.robot.commands.TankDrive;
// import frc.robot.commands.ToggleIntake;
import frc.robot.commands.Auto.Autos;
import frc.robot.commands.Auto.SetIntakeAngle;
import frc.robot.commands.Auto.SetIntakeAngle.Level;
import frc.robot.subsystems.Climber;
// import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
// import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Drivetrain drivetrain = new Drivetrain();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  private final Climber climber = new Climber();

  private final CommandPS5Controller driver = new CommandPS5Controller(ControllerPorts.kDriverControllerPort);
  private final CommandXboxController operator = new CommandXboxController(ControllerPorts.kOperatorControllerPort);

  private SendableChooser<Command> autonomousChooser = new SendableChooser<>();

  public RobotContainer() {
    SmartDashboard.putData(autonomousChooser);
    
    autonomousChooser.setDefaultOption("PID test", Autos.PIDdriveTest(drivetrain, 5));
    autonomousChooser.setDefaultOption("PathPlanner test", Autos.PathPlannerTest(drivetrain));
    autonomousChooser.addOption("Leave", Autos.Leave(drivetrain, 2));
    
    NamedCommands.registerCommand("ActivateIntake", new ActivateIntake(intake).withTimeout(0.5));
    NamedCommands.registerCommand("CollectNote", Autos.CollectNote(intake));
    NamedCommands.registerCommand("ShootNote", new ActivateShooter(shooter).withTimeout(0.5));
    
    registerPathPlanner();
    configureDriverBindings();
    configureOperatorBindings();
    configureDefaultCommands();
  }

  //TODO: get the proper buttons, since these are placeholders
  private void configureDriverBindings() {
    // driver.button(0).toggleOnTrue();

    if (Robot.isReal()){
      driver.R2().onTrue(Autos.FaceSpeaker(drivetrain));
      driver.button(0).whileTrue(new ActivateClimber(climber, 1));
      driver.button(1).whileTrue(new ActivateClimber(climber, -1));
    }
  }

  private void configureOperatorBindings(){
    if (Robot.isSimulation()){
      operator.button(1).onTrue(new SetIntakeAngle(intake, Level.GROUND));
      operator.button(2).onTrue(new SetIntakeAngle(intake,Level.AMP));
      operator.button(3).onTrue(new SetIntakeAngle(intake, Level.PASSOVER));

      operator.button(4).whileTrue(new ActivateShooter(shooter));

      operator.button(5).whileTrue(new ActivateClimber(climber, 1));
      operator.button(6).whileTrue(new ActivateClimber(climber, -1));
      return;
    } 
    // operator.leftBumper().whileTrue(new ActivateShooter(shooter));
    operator.leftBumper().onTrue(Autos.CollectNote(intake));
    operator.rightBumper().whileTrue(new ActivateIntake(intake));
        
    operator.a().onTrue(new SetIntakeAngle(intake, Level.GROUND));
    operator.b().onTrue(new SetIntakeAngle(intake, Level.AMP));
    operator.x().onTrue(new SetIntakeAngle(intake, Level.PASSOVER));
  }

  private void configureDefaultCommands(){
    drivetrain.setDefaultCommand(new TankDrive(drivetrain, driver));
    intake.setDefaultCommand(new PivotIntake(intake, operator));
  }

  public Command getAutonomousCommand() {
    return autonomousChooser.getSelected();
  }

  public void registerPathPlanner(){

  }
}