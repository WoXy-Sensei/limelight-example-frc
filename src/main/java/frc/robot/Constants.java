package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.config.SwerveModuleConstants;

public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class LimelightConstants{
    public static final int pipeNu_megatag = 0;
    public static final int pipeNu_node18_april = 1 ;
    public static final int pipeNu_node27_april = 2 ;
    public static final int pipeNu_node36_april = 3 ;
    public static final int pipeNu_hp45_april = 4;
    public static final int pipeNu_lower_reflective = 5;
    public static final int pipeNu_higher_reflective = 6;
    public static final int pipeNu_normal = 7;
    public static final double higher_reflective_heightCm = 111.0;
    public static final double lower_reflective_heightCm = 61.0;
    public static final double node_april_heightCm = 46.0;
    public static final double hp_april_heightCm = 69.0;
    public static final double limelightMountAngleDegrees = -19.5;
    public static final double limelightLensHeightCm = 107.0;

  }

  public static final class SwerveConstants {
    public static final double stickDeadband = 0.1;
    public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(0);
    public static final double wheelBase = Units.inchesToMeters(0);
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (6.75 / 1.0); 
    public static final double angleGearRatio = (21.42 / 1.0); 

    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 80;


    /* Angle Motor PID Values */
    public static final double angleKP = 0.01;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0001;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.1;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;



    /* Drive Motor PID Values Limelight */
    public static final double LLdriveKP = 0.15;
    public static final double LLdriveKI = 0.0;
    public static final double LLdriveKD = 0.0;
    public static final double LLdriveKFF = 0.05;

    /* Strafe Motor PID Values Limelight */
    public static final double LLstrafeKP = 0.15;
    public static final double LLstrafeKI = 0.0;
    public static final double LLstrafeKD = 0.0;
    public static final double LLstrafeKFF = 0.05;

    /* Angle Motor PID Values Limelight */
    public static final double LLangleKP = 0.02;
    public static final double LLangleKI = 0.0;
    public static final double LLangleKD = 0.001;
    public static final double LLangleKFF = 0.01;


    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.0; 
    public static final double driveKA = 0.2; 



    /* Drive Motor Conversion Factors */
    public static final double driveReductionMK4I = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public static final double steerReductionMK4I = (14.0 / 50.0) * (10.0 / 60.0);
    public static final double driveConversionPositionFactor =  (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 1.5; 
    public static final double maxAngularVelocity = 3; 

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake; 
    
    /* Motor Inverts */
    public static final boolean driveInvert = false; 
    public static final boolean angleInvert = false; 

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false; // Always ensure canCoder is CCW+ CW-
    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 3; 
      public static final int angleMotorID = 2;
      public static final int canCoderID = 1;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(285.90); 
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 6;
      public static final int angleMotorID = 5;
      public static final int canCoderID = 4;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(146.30+180); 
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 12;
      public static final int angleMotorID = 11;
      public static final int canCoderID = 10;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(294.10-180); 
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 9;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 7;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(245.15-180); 
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  }


}