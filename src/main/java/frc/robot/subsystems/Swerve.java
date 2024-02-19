package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
//import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class Swerve extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    public Swerve(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        // for (int i = 0; i < 4; i++) {
        //     SwerveModule s = this.getModule(i);
        //     s.getDriveMotor().setNeutralMode(NeutralModeValue.Coast);
        //     s.getSteerMotor().setNeutralMode(NeutralModeValue.Coast);
        // }
    }
    public Swerve(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        // for (int i = 0; i < 4; i++) {
        //     SwerveModule s = this.getModule(i);
        //     s.getDriveMotor().setNeutralMode(NeutralModeValue.Coast);
        //     s.getSteerMotor().setNeutralMode(NeutralModeValue.Coast);
        // }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public ParentDevice[] requestOrchDevices() {
        ParentDevice[] pd = {
            this.getModule(0).getDriveMotor(),this.getModule(0).getSteerMotor(),this.getModule(0).getCANcoder(),
            this.getModule(1).getDriveMotor(),this.getModule(1).getSteerMotor(),this.getModule(1).getCANcoder(),
            this.getModule(2).getDriveMotor(),this.getModule(2).getSteerMotor(),this.getModule(2).getCANcoder(),
            this.getModule(3).getDriveMotor(),this.getModule(3).getSteerMotor(),this.getModule(3).getCANcoder(),
            this.getPigeon2()
        };
        return pd;
    }
}
