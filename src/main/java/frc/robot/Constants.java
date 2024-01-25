// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class ControllerPorts {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  //TODO: use rev software to set port numbers
  public static class DrivetrainMotorPorts{
    public static final int kBLmotor = 1;
    public static final int kBRmotor = 2;
    public static final int kFLmotor = 3;
    public static final int kFRmotor = 4;
  }
  public static class DrivetrainMechanism{
    public static final double wheelRadius = 1;
    public static final double gearBoxRatio = 1;
  }
  public static class IntakeMotorPorts{
    public static final int kIntakeWheel = 5;
    public static final int kIntakePivot = 6;
  }
  public static class IntakeMechanism{
    //output vs input 
    public static final double pivotGearRatio = 1/(double)12 ;
  }

  public static class ShooterMotorPorts{
    public static final int shooterWheel = 7;
  }

  public static class ClimberMotorPorts{
    public static final int climberMotor = 8;
  }
}
