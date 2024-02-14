// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
	/* CONSTANTS (prefix: c) */
    private double c_MaxSwerveSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  	private double c_MaxSwerveAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

	/* SUBSYSTEM DEFINITIONS (prefix: s) */
	private final Swerve s_Swerve = TunerConstants.DriveTrain;

	/* INPUT DEVICES (prefix: xb) */
	private final Joystick xb_Driver = new Joystick(0);

	/* CONTROL AXES (prefix: axis) */
	private final double axis_Forward = XboxController.Axis.kLeftY.value;
	private final double axis_Strafe  = XboxController.Axis.kLeftX.value;
	private final double axis_Rotate  = XboxController.Axis.kRightX.value;	

	/* CONTROL BUTTONS (prefix: ctrl) */
	private final JoystickButton ctrl_Brake 		= new JoystickButton(xb_Driver, XboxController.Button.kB.value);
	private final JoystickButton ctrl_Aim		   	= new JoystickButton(xb_Driver, XboxController.Button.kY.value);
	private final JoystickButton ctrl_ResetHeading	= new JoystickButton(xb_Driver, XboxController.Button.kLeftBumper.value);

  	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      	.withDeadband(c_MaxSwerveSpeed * 0.1).withRotationalDeadband(c_MaxSwerveAngularRate * 0.1) // Add a 10% deadband
      	.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  	private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  	private final Telemetry logger = new Telemetry(c_MaxSwerveSpeed);

  	private void configureBindings() {
    	s_Swerve.setDefaultCommand( // Drivetrain will execute this command periodically
        	s_Swerve.applyRequest(() -> drive.
				withVelocityX(-axis_Forward * c_MaxSwerveSpeed) // Drive forward with negative Y (forward)
            	.withVelocityY(-axis_Strafe * c_MaxSwerveSpeed) // Drive left with negative X (left)
            	.withRotationalRate(-axis_Rotate * c_MaxSwerveAngularRate) // Drive counterclockwise with negative X (left)
        ));

    	ctrl_Brake.whileTrue(s_Swerve.applyRequest(() -> brake));
    	ctrl_Aim.whileTrue(s_Swerve
        	.applyRequest(() -> point.withModuleDirection(new Rotation2d(-axis_Forward, -axis_Strafe))));

 	   // reset the field-centric heading on left bumper press
    	ctrl_ResetHeading.onTrue(s_Swerve.runOnce(() -> s_Swerve.seedFieldRelative()));
    	if (Utils.isSimulation()) {
			s_Swerve.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    	}
    	s_Swerve.registerTelemetry(logger::telemeterize);
  	}

  	public RobotContainer() {
    	configureBindings();
  	}

  	public Command getAutonomousCommand() {
    	return Commands.print("No autonomous command configured");
  	}
}
