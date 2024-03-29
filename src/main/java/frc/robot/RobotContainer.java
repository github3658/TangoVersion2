// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// This is the primary file. We reference our subsystems here, give them commands, and that's it.

package frc.robot;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class RobotContainer {

	/* CONSTANTS (prefix: c) */
	
	/* SUBSYSTEM DEFINITIONS (prefix: s) */
	private final Swerve   s_Swerve   = TunerConstants.DriveTrain;
	private final Shooter  s_Shooter  = new Shooter();
	private final Intake   s_Intake   = new Intake();
	private final Climber  s_Climber  = new Climber();

	/* INPUT DEVICES (prefix: xb) */
	private final GenericHID xb_Driver = new GenericHID(0);
	private final GenericHID xb_Operator = new GenericHID(1);

	/* CONTROLS (prefix: ctrl) */
	private final int ctrl_ShooterMain = XboxController.Axis.kRightTrigger.value;
	private final int ctrl_ShooterAmp = XboxController.Axis.kLeftTrigger.value;

	/* OTHER VARIABLES */
	private final Orchestra o_Orchestra = new Orchestra();
	private boolean b_Shot = false;
	private boolean b_PlaySong = true;
	private SendableChooser<Command> m_AutoChooser;
  	//private final Telemetry logger = new Telemetry(c_MaxSwerveSpeed); This telemetry is a little excessive at the moment, I think it's better to have just the important info in SmartDashboard.

  	private void setDefaultSubsystemCommands() {
		// These commands contain isolated subsystem behavior
		s_Swerve.setDefaultCommand(new SwerveTeleop(s_Swerve,	 xb_Driver));
		s_Intake.setDefaultCommand(new IntakeTeleop(s_Intake,	 xb_Operator));
		s_Shooter.setDefaultCommand(new ShooterTeleop(s_Shooter, xb_Operator));
		s_Climber.setDefaultCommand(new ClimberTeleop(s_Climber, xb_Driver));

		// More complex behaviors are handled in TeleopPeriodic.
  	}

	private void createNamedCommands() {
		NamedCommands.registerCommand("MetalCrusher",new PlaySong(o_Orchestra, s_Swerve, s_Intake, s_Shooter, "metalcrusher.chrp", xb_Operator));
		NamedCommands.registerCommand("ShootSpeaker",new ShootGeneric(s_Shooter, s_Intake, 1.0));
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
    	setDefaultSubsystemCommands();

		createNamedCommands();

		m_AutoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData(m_AutoChooser);
  	}

  	public Command getAutonomousCommand() {
    	return Commands.print("No autonomous command configured");
  	}

	public void autonomousInit() {
		// NetworkTableInstance inst = NetworkTableInstance.getDefault();
		// NetworkTable nt_db = inst.getTable("DB");
    	// System.out.println(nt_db.getValue("String 0").getString());

		// Command auto = new PathPlannerAuto("!TEST AUTO");
		// auto.schedule();
		SequentialCommandGroup auto = new SequentialCommandGroup(new ShootGeneric(s_Shooter, s_Intake, 0.6), new DriveForwardWorkaround(s_Swerve));
		auto.schedule();
	}

	public void teleopPeriodic() {
		// Reset FOC
		if (xb_Driver.getRawButton(XboxController.Button.kLeftStick.value)) {
			s_Swerve.zeroHeading();
		}

		// Song Selection
		if (!b_PlaySong && xb_Driver.getRawButton(XboxController.Button.kStart.value)) {
			if (xb_Driver.getPOV() == 0) {
				b_PlaySong = true;
				new PlaySong(o_Orchestra, s_Swerve, s_Intake, s_Shooter, "bohemianrhapsody.chrp", xb_Driver).schedule();
			}
			else if (xb_Driver.getPOV() == 45) {
				b_PlaySong = true;
				new PlaySong(o_Orchestra, s_Swerve, s_Intake, s_Shooter, "creep.chrp", xb_Driver).schedule();
			}
			else if (xb_Driver.getPOV() == 90) {
				b_PlaySong = true;
				new PlaySong(o_Orchestra, s_Swerve, s_Intake, s_Shooter, "rickroll.chrp", xb_Driver).schedule();
			}
			else if (xb_Driver.getPOV() == 135) {
				b_PlaySong = true;
				new PlaySong(o_Orchestra, s_Swerve, s_Intake, s_Shooter, "snitch.chrp", xb_Driver).schedule();
			}
			else if (xb_Driver.getPOV() == 180) {
				b_PlaySong = true;
				new PlaySong(o_Orchestra, s_Swerve, s_Intake, s_Shooter, "starwars.chrp", xb_Driver).schedule();
			}
			else if (xb_Driver.getPOV() == 270) {
				b_PlaySong = true;
				new PlaySong(o_Orchestra, s_Swerve, s_Intake, s_Shooter, "metalcrusher.chrp", xb_Driver).schedule();
			}
		}
		else if (xb_Driver.getRawButton(XboxController.Button.kStart.value) == false) {
			b_PlaySong = false;
		}

		// Shoot for the Speaker
		if (xb_Operator.getRawAxis(ctrl_ShooterMain) > 0.9 && !b_Shot) {
			b_Shot = true;
			new ShootGeneric(s_Shooter, s_Intake, 0.60).schedule();
		}
		else if (xb_Operator.getRawAxis(ctrl_ShooterMain) < 0.1 && b_Shot) {
			b_Shot = false;
		}

		// Shoot for the Amp
		if (xb_Operator.getRawAxis(ctrl_ShooterAmp) > 0.9 && !b_Shot) {
			b_Shot = true;
			new ShootGeneric(s_Shooter, s_Intake, 0.13).schedule();
		}
		else if (xb_Operator.getRawAxis(ctrl_ShooterAmp) < 0.1 && b_Shot) {
			b_Shot = false;
		}
	}
}
