// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerPorts;
import frc.robot.commands.ActivateClimber;
import frc.robot.commands.ActivateIntake;
import frc.robot.commands.ActivateShooter;
import frc.robot.commands.PivotIntake;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
// import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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

  private final CommandXboxController driver = new CommandXboxController(ControllerPorts.kDriverControllerPort);
  // private final CommandPS5Controller driver = new CommandPS5Controller(ControllerPorts.kDriverControllerPort);
  private final CommandJoystick operator = new CommandJoystick(ControllerPorts.kOperatorControllerPort);

  public RobotContainer() {
    configureDriverBindings();
    configureOperatorBindings();
    configureDefaultCommands();
  }

  private void configureDriverBindings() {
    // driver.b().
  }

  private void configureOperatorBindings(){
    //TODO: get the proper buttons, since these are placeholders
    // operator.button(0).onTrue(new ToggleIntake(intake));
    operator.button(1).whileTrue(new ActivateShooter(shooter));
    operator.button(0).whileTrue(new ActivateIntake(intake));

    //PIVOT PRESET CONTROL
    operator.button(2).onTrue(new SetIntakeAngle(intake, Level.GROUND));
    operator.button(3).onTrue(new SetIntakeAngle(intake, Level.AMP));
    operator.button(4).onTrue(new SetIntakeAngle(intake, Level.PASSOVER));

    operator.button(5).whileTrue(new ActivateClimber(climber, 1)).onFalse(new InstantCommand( ()->{
      climber.activateClimber(0, 0);
    }));
    operator.button(6).whileTrue(new ActivateClimber(climber, -1)).onFalse(new InstantCommand( ()->{
      climber.activateClimber(0, 0);
    }));
    // operator.button(2).onTrue();

  }

  private void configureDefaultCommands(){
    drivetrain.setDefaultCommand(new TankDrive(drivetrain, driver));
    intake.setDefaultCommand(new PivotIntake(intake, operator));
    // climber.setDefaultCommand(new );
  }

  public Command getAutonomousCommand() {
    return Autos.Leave(drivetrain,1);
  }
}