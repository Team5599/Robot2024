// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;

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
    //input to output
    public static final double kGearBoxRatio = 8.46;

    //TODO: confirm with 2023 and 2024 robot and wpi lib that the following values are accurate
    // public static final double kDriveBaseWeight = 22.6796; // 50 lbs.
    public static final double kDriveBaseWeight = Units.lbsToKilograms(125); // 50 lbs.
    public static final double kWheelDiameter = 6; // inches
    public static final double kWheelTrackWidth = 18; // inches
    // public static final double kCenterMomentInertia = 5.1349; // kg/m^2
    public static final double kCenterMomentInertia = 7; // kg/m^2
    public static final int kGearboxMotorCount = 2;

    public static final DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(kWheelTrackWidth);
    public static final double kPositionConversionFactor = -1 * Math.PI * Units.inchesToMeters(kWheelDiameter); //meters
    public static final double kVelocityConversionFactor = -1 * Math.PI * Units.inchesToMeters(kWheelDiameter) / 60; //meters per second
    // public static final double kWheelVelocityConversionFactor = kVelocityConversionFactor
    public static final double ramseteBeta = 1.2;
    public static final double ramseteZeta = 0.7;

    // public static final DifferentialDrivetrainSim drivetrainSim = new DifferentialDrivetrainSim(
    //   LinearSystemId.createDrivetrainVelocitySystem(
    //     DCMotor.getNEO(2),
    //     kGearBoxRatio,
    //     kWheelDiameter / 2,
    //     kWheelTrackWidth / 2,
    //     kCenterMomentInertia,
    //     kGearBoxRatio
    //   ),
    //   DCMotor.getNEO(4), 
    //   kGearBoxRatio, 
    //   kWheelTrackWidth, 
    //   kCenterMomentInertia, 
    //   null
    // );
    
    // public static final double maxVelocity;
    // public static final double maxAcceleration;
  }

  public static class IntakeMotorPorts{
    public static final int kIntakeWheel = 5;
    public static final int kIntakePivot = 6;
    //TODO: remove if not using a sensor
    public static final int kSensorPort = 0;
  }

  public static class IntakeMechanism{
    //output vs input 
    public static final double pivotGearRatio = 1/(double)12 ;

    //inches
    public static final double intakeMountLength = Units.inchesToMeters(5.75);
    public static final double intakeAntebrachialLength = Units.inchesToMeters(10);
    public static final double intakeCarpalLength = Units.inchesToMeters(8);
    public static final double intakeWristangle = -30;

    // public static final double mechX;
    // public static final double mechY;

    // /https://www.revrobotics.com/rev-21-1650/ for values
    public static final DCMotor intakePivotDC = new DCMotor(
      12,
      2.6,
      105,
      1.8,
      594,
      1
    );

    public static final DCMotor intakePivotGearBox = DCMotor.getNEO(1);
    
    public static final double intakeMOI = 0.002;//moment of inertia
    public static final LinearSystem<N2,N1,N1> intakePlant = LinearSystemId.createSingleJointedArmSystem(
      intakePivotDC, 
      intakeMOI, 
      pivotGearRatio
    );
  }

  public static class ShooterMotorPorts{
    public static final int upperShooterWheels = 7;
    public static final int lowerShooterWheels = 8;
  }

  public static class ShooterMechanism{
    public static final double kWheelDiameter = 4;
    public static final double kPositionConversionFactor = Math.PI * Units.inchesToMeters(kWheelDiameter);
    public static final double kVelocityConversionFactor = Math.PI * Units.inchesToMeters(kWheelDiameter) / 60;
  }

  //USING PWM PORTS
  public static class ClimberMotorPorts{
    public static final int leftClimberMotor = 0;
    public static final int rightClimberMotor = 1;
  }
  
  public static class ClimberMechanism{
    public static final double contractedLength = Units.inchesToMeters(30);
    public static final double extendedLength = Units.inchesToMeters(24.5);
    
    public static final double climberMass = Units.lbsToKilograms(3);
    public static final double climberGearRatio = 1/(double)48;//output vs input
    
    // public static final double mechX;
    // public static final double mechY;
    
    //https://www.andymark.com/products/andymark-775-redline-motor-v2
    public static final DCMotor climberDC = new DCMotor(
      12,//nominal voltage here
      0.7,
      130,
      3.8,
      2201,
      1
    );
    //https://www.andymark.com/products/climber-in-a-box
    public static final LinearSystem<N2,N1,N1> climberPlant = LinearSystemId.createElevatorSystem(
      climberDC,
      climberMass,
      Units.inchesToMeters(2),
      climberGearRatio
    );
  }

  public static class Pipelines{
    //we have a total of 10 different pipelines
    public static int NOTE = 0;
    //uses apriltags 6 7 8 9 10 and 15 16

    //TODO: make a pipeline specifically for note intaking
    public static class RED{
      public static int SPEAKER = 1;
    }
    //uses apriltags 1 2 3 4 5 and 11 12
    public static class BLUE{
      public static int SPEAKER = 2;
    }
  }
}
