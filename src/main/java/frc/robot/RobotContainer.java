// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class RobotContainer {

	/* CONSTANTS (prefix: c) */
	private double c_MaxSwerveSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed

	/* SUBSYSTEM DEFINITIONS (prefix: s) */
	private final Swerve   s_Swerve   = TunerConstants.DriveTrain;
	private final Shooter  s_Shooter  = new Shooter();
	private final Intake   s_Intake   = new Intake();
	private final Climber  s_Climber  = new Climber();

	/* INPUT DEVICES (prefix: xb) */
	private final GenericHID xb_Driver = new GenericHID(0);

	/* CONTROL AXES (prefix: axis) */
	private final int axis_Forward = XboxController.Axis.kLeftY.value;
	private final int axis_Strafe  = XboxController.Axis.kLeftX.value;
	private final int axis_Rotate  = XboxController.Axis.kRightX.value;

	/* CONTROL BUTTONS (prefix: ctrl) */
	private final JoystickButton ctrl_Brake 		= new JoystickButton(xb_Driver, XboxController.Button.kB.value);
	private final JoystickButton ctrl_Aim		   	= new JoystickButton(xb_Driver, XboxController.Button.kY.value);
	private final JoystickButton ctrl_ResetHeading	= new JoystickButton(xb_Driver, XboxController.Button.kLeftBumper.value);
	private final JoystickButton ctrl_BohemianRhapsody = new JoystickButton(xb_Driver, XboxController.Button.kRightStick.value);

	/* OTHER VARIABLES */
	private final Orchestra o_Orchestra = new Orchestra();

  	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  	private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  	private final Telemetry logger = new Telemetry(c_MaxSwerveSpeed);

  	private void configureBindings() {
    	s_Swerve.setDefaultCommand(new SwerveDriveRamped(s_Swerve));

    	ctrl_Brake.whileTrue(s_Swerve.applyRequest(() -> brake));
    	ctrl_Aim.whileTrue(s_Swerve
        	.applyRequest(() -> point.withModuleDirection(new Rotation2d(-axis_Forward, -axis_Strafe))));

 	   // reset the field-centric heading on left bumper press
    	ctrl_ResetHeading.onTrue(s_Swerve.runOnce(() -> s_Swerve.seedFieldRelative()));
    	if (Utils.isSimulation()) {
			s_Swerve.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    	}
    	s_Swerve.registerTelemetry(logger::telemeterize);

		ctrl_BohemianRhapsody.onTrue(new PlaySong(o_Orchestra, "bohemianrhapsody.chrp"));
  	}

  	public RobotContainer() {
		// Init orchestra
		for (ParentDevice pd : s_Climber.requestOrchDevices()) {
			o_Orchestra.addInstrument(pd);
		}
		for (ParentDevice pd : s_Intake.requestOrchDevices()) {
			o_Orchestra.addInstrument(pd);
		}
		for (ParentDevice pd : s_Shooter.requestOrchDevices()) {
			o_Orchestra.addInstrument(pd);
		}
		for (ParentDevice pd : s_Swerve.requestOrchDevices()) {
			o_Orchestra.addInstrument(pd);
		}
    	configureBindings();
  	}

  	public Command getAutonomousCommand() {
    	return Commands.print("No autonomous command configured");
  	}
}
