package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;

public class DriveForwardWorkaround extends Command {
    private final Swerve s_Swerve;

    private final double c_MaxSwerveSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  	private final double c_MaxSwerveAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
    private int timer = 50*4;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    //.withDeadband(c_MaxSwerveSpeed * 0.1).withRotationalDeadband(c_MaxSwerveAngularRate * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

    public DriveForwardWorkaround(Swerve subsystem) {
        s_Swerve = subsystem;
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
        timer = 50*5;
    }

    @Override
    public void execute() {
        double forward = 0.2;
        double strafe = 0;
        double rotate = 0;

        s_Swerve.setControl(drive.
            withVelocityX(forward * c_MaxSwerveSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(strafe * c_MaxSwerveSpeed) // Drive left with negative X (left)
            .withRotationalRate(rotate * c_MaxSwerveAngularRate) // Drive counterclockwise with negative X (left)
        );
        timer--;
    }

    @Override
    public boolean isFinished() {
        return timer < 0;
    }    
}
