package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;

public class SwerveTeleop extends Command {
    private final Swerve s_Swerve;

    private final double c_MaxSwerveSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  	private final double c_MaxSwerveAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
    private final double c_SwerveRampDeadzone = 0.05;
    private final int c_AccelTime = 50;
    private double d_SwerveRamp = 0.0;

    private GenericHID xb_Driver;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(c_MaxSwerveSpeed * 0.1).withRotationalDeadband(c_MaxSwerveAngularRate * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

    public SwerveTeleop(Swerve subsystem, GenericHID port0) {
        s_Swerve = subsystem;
        xb_Driver = new GenericHID(0);
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double forward = 0.05;//-Math.pow(xb_Driver.getRawAxis(XboxController.Axis.kLeftY.value),3);
        double strafe = -Math.pow(xb_Driver.getRawAxis(XboxController.Axis.kLeftX.value),3);
        double rotate = -Math.pow(xb_Driver.getRawAxis(XboxController.Axis.kRightX.value),3);

        if (Math.abs(forward) > c_SwerveRampDeadzone || Math.abs(strafe) > c_SwerveRampDeadzone || Math.abs(rotate) > c_SwerveRampDeadzone) {
            d_SwerveRamp = Math.min(d_SwerveRamp+1/c_AccelTime,1);
        }
        else {
            d_SwerveRamp = Math.max(d_SwerveRamp-1/c_AccelTime,0);
        }

        s_Swerve.applyRequest(() -> 
            drive.
            withVelocityX(forward * d_SwerveRamp * c_MaxSwerveSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(strafe * d_SwerveRamp * c_MaxSwerveSpeed) // Drive left with negative X (left)
            .withRotationalRate(rotate * d_SwerveRamp * c_MaxSwerveAngularRate) // Drive counterclockwise with negative X (left)
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}