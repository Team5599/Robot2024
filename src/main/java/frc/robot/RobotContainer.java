// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerPorts;
import frc.robot.commands.PivotIntake;
import frc.robot.commands.TankDrive;
import frc.robot.commands.ToggleIntake;
import frc.robot.commands.Auto.Autos;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Drivetrain drivetrain = new Drivetrain();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driver = new CommandXboxController(ControllerPorts.kDriverControllerPort);
  private final CommandJoystick operator = new CommandJoystick(ControllerPorts.kOperatorControllerPort);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configureDefaultCommands();
  }

  private void configureBindings() {
    // driver.b().
    operator.button(0).onTrue(new ToggleIntake(intake));
  }

  private void configureDefaultCommands(){
    drivetrain.setDefaultCommand(new TankDrive(drivetrain, driver));
    intake.setDefaultCommand(new PivotIntake(intake, operator));
  }

  public Command getAutonomousCommand() {
    return Autos.Leave(drivetrain,1);
  }
}