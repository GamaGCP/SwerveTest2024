package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase {
    private ShuffleboardTab modualTab = Shuffleboard.getTab("Modual Info");
    private ShuffleboardTab swerveTab = Shuffleboard.getTab("SDS Swerve");

    //shuffleboard telementry
    private GenericEntry xSpeedEntry =
    swerveTab.add("Controler xSpeed", 0).getEntry();

    private GenericEntry ySpeedEntry =
    swerveTab.add("Controler ySpeed", 0).getEntry();

    private GenericEntry rotSpeedEntry =
    swerveTab.add("Controler rotSpeed", 0).getEntry();
    

    private SwerveModule[] mSwerveMods;

    private SwerveDrivePoseEstimator driveOdometry;

    private Field2d field;

    private final AHRS gyro;

//create the swereve moduals
    public SwerveSubsystem()
    {

        gyro = new AHRS();
        zeroGyro();
    mSwerveMods =
        new SwerveModule[] 
        {
          new SwerveModule(0, Constants.DriveConstants.mod0.constants, modualTab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0)),
          new SwerveModule(1, Constants.DriveConstants.mod1.constants, modualTab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 4)),
          new SwerveModule(2, Constants.DriveConstants.mod2.constants, modualTab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0)),
          new SwerveModule(3, Constants.DriveConstants.mod3.constants, modualTab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 4))
        };

        
    resetToAbsolute2();

    //standerd deviations for pose estimator-how much we trust either thing
    var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    var visionStdDevs = VecBuilder.fill(1, 1, 1);

     this.driveOdometry = 
      new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, getYaw(), getModulePositions(), new Pose2d(), stateStdDevs, visionStdDevs); //add a function to get starting pose?
  
      field = new Field2d();
    SmartDashboard.putData("Field", field);

    //Create PathPlanner AutoBuilder

    AutoBuilder.configureHolonomic(this::getPose,this::resetOdometry, this::getRobotRelevtiveSpeeds, this::setModuleStatesFromChassisSpeed, Constants.ModuleConstants.HolonomicConfig, this);

    //add path to smart dashboard
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
   
    }
    public void resetToAbsolute2(){
        for(SwerveModule mod : mSwerveMods){
          mod.resetToAbsolute();
        }
      }
       public Rotation2d getYaw() {
    return (Constants.DriveConstants.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw())
        : Rotation2d.fromDegrees(gyro.getYaw());
  } 

  public void zeroGyro() {
    gyro.reset();
  }

      public SwerveModulePosition[] getModulePositions() 
      {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods)
        {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
      }

      public SwerveModuleState[] getModuleStates() 
      {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods)
        {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
      }

    public Pose2d getPose() 
      {
        return driveOdometry.getEstimatedPosition();
      }

      public void resetOdometry(Pose2d pose){
        driveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
        
      }

      public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative)
      {
        var swerveModuleStates = 
          DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getYaw())
              : new ChassisSpeeds(xSpeed, ySpeed, rot));
        setModuleStates(swerveModuleStates);
    
        // Telemetry
        xSpeedEntry.setDouble(xSpeed);
        ySpeedEntry.setDouble(ySpeed);
        rotSpeedEntry.setDouble(rot);
            
      }

      public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
          desiredStates, DriveConstants.kPhysicalMaxSpeedMeterPerSecond);
          for (SwerveModule mod : mSwerveMods) 
           {
            mod.SetDesiredState(desiredStates[mod.moduleNumber]);
           }
      }

      public void setModuleStatesFromChassisSpeed(ChassisSpeeds robotReletiveSpeeds) {
        var targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(robotReletiveSpeeds);
        
        SwerveDriveKinematics.desaturateWheelSpeeds(
          targetStates, DriveConstants.kPhysicalMaxSpeedMeterPerSecond);
          for (SwerveModule mod : mSwerveMods) 
           {
            mod.SetDesiredState(targetStates[mod.moduleNumber]);
           }
      }

      public ChassisSpeeds getRobotRelevtiveSpeeds(){
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
      }

      public void resetEncoders(){
        for (SwerveModule mod : mSwerveMods){
            mod.resetDriveEncoders();
            mod.resetTurnEncoders();
        }
      }

      public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(gyro.getYaw());
    }

      public void stopModules(){
        for (SwerveModule mod : mSwerveMods){
            mod.stop();
        }
      }

      public double getTurnRate(){
        return gyro.getRate() * (DriveConstants.invertGyro ? -1.0 : 1.0);
      }
    public void updateOdometry() {
    driveOdometry.update(getYaw(), getModulePositions());
  }

   public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
        driveOdometry.addVisionMeasurement(visionMeasurement, timestampSeconds);
    }

    
    public void addVisionMeasurement(
            Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        driveOdometry.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
    }

     

  @Override
  public void periodic() 
        {
        updateOdometry();
        field.setRobotPose(getPose());
    
        for (SwerveModule mod : mSwerveMods) 
            {
    
          SmartDashboard.putNumber(
              "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoderRot2D().getDegrees());
          SmartDashboard.putNumber(
              "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
          SmartDashboard.putNumber(
              "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
         
            }
            
            
        
             
        }
      }
