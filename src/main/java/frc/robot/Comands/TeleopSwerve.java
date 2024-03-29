package frc.robot.Comands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Vision;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Subsystems.SwerveSubsystem;

public class TeleopSwerve extends Command {

    private final SwerveSubsystem swerveSubsystem;
    // private final Vision visionSubSystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOriented;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private boolean isFieldOriented = false;
  
    public TeleopSwerve(
    SwerveSubsystem swerveSubsystem,
    Supplier<Double> xSpdFunction,
    Supplier<Double> ySpdFunction,
    Supplier<Double> turningSpdFunction,
    Supplier<Boolean> fieldOrientedFunction)//,
    // Vision visionSubSystem) 
    {
        this.swerveSubsystem = swerveSubsystem;
        // this.visionSubSystem = visionSubSystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOriented = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
        // addRequirements(visionSubSystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
          // 1. Get real-time joystick inputs
          double xSpeed = xSpdFunction.get();
          double ySpeed = ySpdFunction.get();
          double turningSpeed = turningSpdFunction.get();
  
          // 2. Apply deadband
          xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
          ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
          turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband/2 ? turningSpeed : 0.0;
  
          // 3. Make the driving smoother
          xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
          ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
          turningSpeed = turningLimiter.calculate(turningSpeed)
                  * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
  
                 isFieldOriented = !fieldOriented.get();
          // 4. Construct desired chassis speeds
          swerveSubsystem.drive(xSpeed, ySpeed, turningSpeed,isFieldOriented);
          // 7. Telemetry
          SmartDashboard.putNumber("xComponent", xSpeed);
          SmartDashboard.putNumber("yComponent", ySpeed);
          SmartDashboard.putNumber("rotationComponent", turningSpeed);

          //vision
        //   var visionEst = visionSubSystem.getEstimatedGlobalPose();
        //            visionEst.ifPresent(
        //                    est -> {
        //                        var estPose = est.estimatedPose.toPose2d();
        //                        // Change our trust in the measurement based on the tags we can see
        //                        var estStdDevs = visionSubSystem.getEstimationStdDevs(estPose);
           
        //                        swerveSubsystem.addVisionMeasurement(
        //                                est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
        //                    });
           
                   // Apply a random offset to pose estimator to test vision correction // MOVE TO ROBOTCONTAINER
                   // if (controller.getBButtonPressed()) {
                   //     var trf =
                   //             new Transform2d(
                   //                     new Translation2d(rand.nextDouble() * 4 - 2, rand.nextDouble() * 4 - 2),
                   //                     new Rotation2d(rand.nextDouble() * 2 * Math.PI));
                   //     drivetrain.resetPose(drivetrain.getPose().plus(trf), false);
                   // }
           
                   
    }   

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}

