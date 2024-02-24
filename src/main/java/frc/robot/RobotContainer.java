// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
//import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
	//private final Climber  s_Climber  = new Climber();

	private final ShootGeneric com_ShootFull = new ShootGeneric(s_Shooter, s_Intake, 0.6);

	/* INPUT DEVICES (prefix: xb) */
	private final GenericHID xb_Driver = new GenericHID(0);
	private final GenericHID xb_Operator = new GenericHID(1);

	/* CONTROL AXES (prefix: axis) */
	private JoystickButton ctrl_TeleopShoot = new JoystickButton(xb_Operator, XboxController.Axis.kRightTrigger.value);

	/* CONTROL BUTTONS (prefix: ctrl) */
	private JoystickButton ctrl_tempsoot = new JoystickButton(xb_Operator, XboxController.Button.kB.value);
	private JoystickButton ctrl_AmpShooter = new JoystickButton(xb_Operator, XboxController.Button.kA.value);
	private JoystickButton ctrl_ZeroIntake = new JoystickButton(xb_Operator, XboxController.Button.kStart.value);
	private JoystickButton ctrl_BohemianRhapsody = new JoystickButton(xb_Operator, XboxController.Button.kLeftStick.value);

	/* OTHER VARIABLES */
	private final Orchestra o_Orchestra = new Orchestra();
	private boolean b_Shot = false;

  	//private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  	//private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  	private final Telemetry logger = new Telemetry(c_MaxSwerveSpeed);

  	private void configureBindings() {
		// These commands contain isolated subsystem behavior
    	//s_Swerve.setDefaultCommand(new SwerveTeleop(s_Swerve,xb_Driver));
		s_Swerve.setDefaultCommand(new SwerveTeleop(s_Swerve,	xb_Driver));
		s_Intake.setDefaultCommand(new IntakeTeleop(s_Intake,	xb_Operator));
		s_Shooter.setDefaultCommand(new ShooterTeleop(s_Shooter,xb_Operator));

		// These are more complex behaviors that call upon multiple subsystems.

		//ctrl_Shoot.onTrue(new ShootSpeaker(s_Shooter, s_Intake));
		
		ctrl_tempsoot.onTrue(com_ShootFull);
		
		// if (xb_Operator.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.9 && !b_Shot) {
		// 	b_Shot = true;
		// 	com_ShootFull.schedule();
		// }
		// else if (xb_Operator.getRawAxis(XboxController.Axis.kRightTrigger.value) < 0.1 && b_Shot) {
		// 	b_Shot = false;
		// }

		SmartDashboard.putBoolean("Shooting", com_ShootFull.isScheduled());

		ctrl_ZeroIntake.whileTrue(new ZeroIntake(s_Intake));
		//ctrl_BohemianRhapsody.onTrue(new PlaySong(o_Orchestra, s_Swerve, s_Intake, s_Shooter, "starwars.chrp", xb_Operator));

		// if (xb_Operator.getRawButton(XboxController.Button.kRightBumper.value)) {
		// 	ctrl_AmpShooter.onTrue(new ShootGeneric(s_Shooter, s_Intake, 0.10));
		// }

		// if (xb_Operator.getRawButton(XboxController.Button.kBack.value)) {
		// 	if (xb_Operator.getPOV() == 0) {
				
		// 	}
		// }

    	//ctrl_Brake.whileTrue(s_Swerve.applyRequest(() -> brake));
    	//ctrl_Aim.whileTrue(s_Swerve
        //	.applyRequest(() -> point.withModuleDirection(new Rotation2d(-axis_Forward, -axis_Strafe))));

 	   // reset the field-centric heading on left bumper press
    	//ctrl_ResetHeading.onTrue(s_Swerve.runOnce(() -> s_Swerve.seedFieldRelative()));
    	if (Utils.isSimulation()) {
			s_Swerve.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    	}
    	s_Swerve.registerTelemetry(logger::telemeterize);

		//ctrl_BohemianRhapsody.onTrue(new PlaySong(o_Orchestra, "bohemianrhapsody.chrp"));
  	}

  	public RobotContainer() {
		// Init orchestra
		// for (ParentDevice pd : s_Climber.requestOrchDevices()) {
		// 	o_Orchestra.addInstrument(pd);
		// }
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

	//public void teleopPeriodic() {
	//
	//}
}
