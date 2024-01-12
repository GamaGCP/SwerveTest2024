package frc.robot;



import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.Lib.config.SwerveModuleConstants;
import edu.wpi.first.math.Matrix;

public class Constants {

    public static final class ModuleConstants {
        
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
        public static final double kDriveMotorGeatRatio = 1 / 6.75 ;
        public static final double kTurningMotorGearRatio = (1 / (150/7)); // about 1/21.429

        public static final double kDriveEncoderRot2Meter =  kDriveMotorGeatRatio * Math.PI * kWheelDiameterMeters;
        public static final double kDriveEncoderRpm2MeterPerSecond = kDriveEncoderRot2Meter / 60;
        public static final double kTurnEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kTurnEncoderRpm2RadperSecond = kTurnEncoderRot2Rad / 60;

        //Turning PID values

        public static final double kPTurning = 0.003;//FIXME
        public static final double kITurning = 0.0000001;
        public static final double kDturning = 0.0001;

        //Drive Pid Values

        public static final double kPDrive = 0.001;//FIXME
        public static final double kIDrive = 0.0;
        public static final double kDDive = 0.0;

        //Drive FeedForward Values

        public static final double kSDrive = 0.667; //0.667 //Find using SysID
        public static final double kVDrive = 2.44; //2.44
        public static final double kADrive = 0.27; //0.27

        // PathPlaner Configs
        public static final HolonomicPathFollowerConfig HolonomicConfig = new HolonomicPathFollowerConfig(
          new PIDConstants(kPDrive, kIDrive, kDDive),
          new PIDConstants(kPDrive, kIDrive, kDDive),
          DriveConstants.kPhysicalMaxSpeedMeterPerSecond,
          Units.inchesToMeters(15.02),// drive base radius,distance from center to furthest module
        new ReplanningConfig()
        );

    }
    
    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final double kDeadband = 0.05;
      }

    public static final class DriveConstants {

        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW- 

        public static final double kTeleDriveSpeedReduction = 1.2;    // 1/int
        public static final double kTeleAngularSpeedReduction = 1.2;  // 1/int

        // Distance between right and left wheels

        public static final double kTrackWidth = Units.inchesToMeters(21.25);

        // Distance between front and back wheels

        public static final double kWheelBase = Units.inchesToMeters(21.25);

        
        
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2), //Front Left
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),//Front Right
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),//Back Left
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));//Beck Right

           /* Module Specific Constants */
    /* Front left Module - Module 0 */
    public static final class mod0 {
        public static final int driveMotorID = 10;
        public static final int angleMotorID = 11;
        public static final int canCoderID = 2;
        public static final double angleOffset = -34.98046875; //original value 325.019531250, optimized it 
        public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }
  
      /* Back right Module - Module 1 */
      public static final class mod3 {
        public static final int driveMotorID = 9;
        public static final int angleMotorID = 8;
        public static final int canCoderID = 0;
        public static final double angleOffset = -75.673828125; // original value 284.326171875, optimized it
        public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }
  
      /* Back left Module - Module 2 */
      public static final class mod2 {
        public static final int driveMotorID = 12;
        public static final int angleMotorID = 13;
        public static final int canCoderID = 1;
        public static final double angleOffset = 174.8144;
        public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }
  
      /* Front Right Module - Module 3 */
      public static final class mod1 {
        public static final int driveMotorID = 6;
        public static final int angleMotorID = 5;
        public static final int canCoderID = 3;
        public static final double angleOffset = 147.041015625;
        public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
          }


        //physical max speeds

        public static final double kPhysicalMaxSpeedMeterPerSecond = 4.4196; // meters per second IN THEORY
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
        public static final double kPhysicalMaxAngularAccelerationRadiansPerSecond = 2 * Math.PI;

        //Max teleop speeds
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMeterPerSecond / kTeleDriveSpeedReduction;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / kTeleAngularSpeedReduction;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }
    public static final class AutoConstants{
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
        
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class Vision{
      public static final String kCameraName = "camera";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFields.kDefaultField.loadAprilTagLayoutField();

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }
}
