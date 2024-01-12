package frc.robot.Subsystems;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.Lib.config.*;

public class SwerveModule extends SubsystemBase {

    //define the differnt components 
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    private final RelativeEncoder driveEncoder;

    //feedback
    private final ProfiledPIDController  turningPIDControler = new ProfiledPIDController(ModuleConstants.kPTurning, ModuleConstants.kITurning, ModuleConstants.kDturning, new TrapezoidProfile.Constraints(DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond, DriveConstants.kPhysicalMaxAngularAccelerationRadiansPerSecond));
    private final PIDController drivePIDControler = new PIDController(ModuleConstants.kPDrive, ModuleConstants.kIDrive, ModuleConstants.kDDive);

    //feed Forward
    //private final SimpleMotorFeedforward turnFeedForward = new SimpleMotorFeedforward(ModuleConstants.kSTurning, ModuleConstants.kVTurning, ModuleConstants.kATurning);
    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(ModuleConstants.kSDrive, ModuleConstants.kVDrive, ModuleConstants.kADrive);
    
    
    private final CANcoder CANCoder ;

    public int moduleNumber;
    private Rotation2d lastAngle;

   

    

    private CANcoderConfiguration CANCoderConfiguration = new CANcoderConfiguration();
    private MagnetSensorConfigs MagnetConfigs = new MagnetSensorConfigs();
    

   

    public SwerveModule(int moduleNumber, SwerveModuleConstants  moduleConstraints, ShuffleboardLayout container  ) {
        
        this.moduleNumber = moduleNumber;
        this.CANCoder = new CANcoder(moduleConstraints.cancoderID);
        var CANCoderConfiger = this.CANCoder.getConfigurator();
       MagnetConfigs.MagnetOffset = (moduleConstraints.angleOffset/360);
       MagnetConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
       CANCoderConfiguration.withMagnetSensor(MagnetConfigs);
       CANCoderConfiger.apply(MagnetConfigs);
       



        // this.CANCoder.configMagnetOffset(-1 * moduleConstraints.angleOffset); //Possible reversed //FIXME
        // this.CANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        driveMotor = new CANSparkMax(moduleConstraints.driveMotorID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(moduleConstraints.angleMotorID, MotorType.kBrushless);

        //driveMotor.setInverted(driveMotorReversed);  //no use
        //turnMotor.setInverted(turnMotorReversed);    // no use

        driveEncoder = driveMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRpm2MeterPerSecond);
       
        turningPIDControler.enableContinuousInput(-Math.PI, Math.PI);

        resetDriveEncoders();
        this.lastAngle = getState().angle;
    }

    public SwerveModulePosition  getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), getCanCoderRot2D());
    }

    public Rotation2d getCanCoderRot2D() {
       var absolutePosition = CANCoder.getAbsolutePosition();
       var absoluteAngle = absolutePosition.getValue() * 2 * Math.PI;
    
        return Rotation2d.fromRadians(absoluteAngle);
      }

    public double getTurnPosition() { //Module Heading
       var Signal = CANCoder.getAbsolutePosition();
        return (2 * Math.PI) *  Signal.getValue();
    }

    public void resetToAbsolute() {
        lastAngle = Rotation2d.fromDegrees(0);
      }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public void resetDriveEncoders() {
        driveEncoder.setPosition(0.0);
        
    }
   

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getCanCoderRot2D());
    }

    public void SetDesiredState(SwerveModuleState desieredState) {
        var CANSignal = CANCoder.getAbsolutePosition();
        SwerveModuleState state = SwerveModuleState.optimize(desieredState, getState().angle);
        Rotation2d desiredAngle = (Math.abs(desieredState.speedMetersPerSecond) < (Constants.DriveConstants.kPhysicalMaxSpeedMeterPerSecond *0.01)) ? this.lastAngle : state.angle;
        this.lastAngle = desiredAngle;
        SmartDashboard.putNumber("SpeedMetersPerSecond", getDriveVelocity());
        SmartDashboard.putNumber("AbsoluteEncoderAngle", CANSignal.getValueAsDouble());
        SmartDashboard.putNumber("desiredState", state.speedMetersPerSecond);



        final double drivePID = drivePIDControler.calculate(getDriveVelocity(), state.speedMetersPerSecond);
        final var turnOutput = turningPIDControler.calculate(getTurnPosition(), desiredAngle.getRadians());
        final double driveFF = driveFeedForward.calculate(state.speedMetersPerSecond);
        final double driveOutput = drivePID + driveFF;
        driveMotor.setVoltage(driveOutput);
        turnMotor.set(turnOutput);

        SmartDashboard.putNumber("turnPID Setpoint Velocity", turningPIDControler.getSetpoint().velocity);
    SmartDashboard.putNumber("PID driveOutput", driveOutput);
    SmartDashboard.putNumber("PID turnOutput", turnOutput);
    }

    public void stop() {    
        driveMotor.set(0.0);
        turnMotor.set(0.0);
    }
    




}