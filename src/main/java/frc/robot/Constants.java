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
  public static class LED{
    public static final int kLEDPort = 2;
    public static final int kLEDLength = 10;
  }

  public static class ControllerPorts {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static class DrivetrainMotorPorts{
    public static final int kBLmotor = 1;
    public static final int kBRmotor = 2;
    public static final int kFLmotor = 3;
    public static final int kFRmotor = 4;
  }
  public static class DrivetrainMechanism{
    //input to output
    public static final double kGearBoxRatio = 8.46;

    //SIM data
    // public static final double kDriveBaseWeight = 22.6796; // 50 lbs.
    public static final double kDriveBaseWeight = Units.lbsToKilograms(125); // 50 lbs.
    public static final double kWheelDiameter = 6; // inches
    public static final double kWheelTrackWidth = 18; // inches
    // public static final double kCenterMomentInertia = 5.1349; // kg/m^2
    public static final double kCenterMomentInertia = 7; // kg/m^2
    public static final int kGearboxMotorCount = 2;

    public static final DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(kWheelTrackWidth);
    public static final double kPositionConversionFactor = Math.PI * Units.inchesToMeters(kWheelDiameter) / kGearBoxRatio; //meters    
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60; //meters per second

    //TODO: SysID required
    public static final double ramseteBeta = 1.2;
    public static final double ramseteZeta = 0.7;

    public static final double kS = 0.15;
    public static final double kV = 0.3;
    public static final double kA = 0.7;
  }

  public static class IntakePorts{
    public static final int kIntakeWheel = 5;
    public static final int kIntakePivot = 6;
  }

  public static class IntakeMechanism{
    //output vs input 
    public static final double pivotGearRatio = 1/(double)75 ;

    public static final double intakeMountLength = Units.inchesToMeters(5.75);
    public static final double intakeAntebrachialLength = Units.inchesToMeters(10);
    public static final double intakeCarpalLength = Units.inchesToMeters(8);
    public static final double intakeWristangle = -30;

    public static final double kPositionConversionFactor = pivotGearRatio * Units.radiansToDegrees(Math.PI * 2);

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
    // public static final int leftClimberMotor = 0;
    // public static final int rightClimberMotor = 1;
    public static final int leftClimberMotor = 9;
    public static final int rightClimberMotor = 10;
  }
  
  public static class ClimberMechanism{
    public static final double contractedLength = Units.inchesToMeters(30);
    public static final double extendedLength = Units.inchesToMeters(24.5);
    
    public static final double climberMass = Units.lbsToKilograms(3);
    public static final double climberGearRatio = 1/(double)28;//output vs input

    public static final double kRightClimberSpoolRadius = Units.inchesToMeters(2);
    public static final double rightPositionConversionFactor = Math.PI * 2 * kRightClimberSpoolRadius;    
    public static final double kLeftClimberSpoolRadius = Units.inchesToMeters(1.5);
    public static final double leftPositionConservsionFactor = Math.PI * 2 * kLeftClimberSpoolRadius;

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
    public static int STREAM = 0;
    public static int NOTE = 9;
    //uses apriltags 6 7 8 9 10 and 15 16

    public static class RED{
      public static int SPEAKER = 1;
      public static int AMP = 2;
    }
    //uses apriltags 1 2 3 4 5 and 11 12
    public static class BLUE{
      public static int SPEAKER = 3;
      public static int AMP = 4;
    }
  }

  public static class PIDConstants{
    public static class Drive{
      public static final double p = 1.2;
      public static final double i = 0;
      public static final double d = 0;
    }
    public static class Turn{
      public static final double p = 0.075;
      public static final double i = 0.075;
      public static final double d = 0;
    }
    public static class Pivot{
      public static final double p = 0.02;
      public static final double i = 0.02;
      public static final double d = 0.001;
    }
  }
}
