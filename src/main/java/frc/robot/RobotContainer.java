// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;








import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Comands.TeleopSwerve;
import frc.robot.Constants.OIConstants;
import frc.robot.Subsystems.SwerveSubsystem;

public class RobotContainer {
  //Create Swereve subSystem, controler, and buttons
  public final SwerveSubsystem s_Swerve = new SwerveSubsystem();
  // public final Vision s_Vision = new Vision();
  XboxController driverJoystick = new XboxController(OIConstants.kDriverControllerPort);  // USE XI INPUT
  private final JoystickButton zeroGyro = new JoystickButton(driverJoystick, XboxController.Button.kA.value);
  private final JoystickButton robotCentric = new JoystickButton(driverJoystick, XboxController.Button.kLeftBumper.value);

  //create auto chooser
  private final SendableChooser<Command> autoChooser;


    


  public RobotContainer() {

    s_Swerve.setDefaultCommand(
            new TeleopSwerve(
              s_Swerve,
              () -> driverJoystick.getLeftX(),
              () -> driverJoystick.getLeftY(),
              () -> driverJoystick.getRightX(),
              () -> robotCentric.getAsBoolean()
              // s_Vision
              )
    );

    configureBindings();

    //Register commands here for PathPlaner //NamedCommands.registerCommands("exact name in pathplaner", example.command());

    //Build auto chooser
   autoChooser = AutoBuilder.buildAutoChooser("");

    SmartDashboard.putData("Auto Chooser", autoChooser);

    
  }

  private void configureBindings() {
     zeroGyro.onTrue( new InstantCommand(() -> s_Swerve.zeroGyro()));
  }



  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  
}
