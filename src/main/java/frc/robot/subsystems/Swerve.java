// This is the swerve subsystem. I didn't write this code, and you shouldn't have to look through this file either.
// Most swerve functionality is handled through other things.

package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.function.Supplier;
import java.util.function.Consumer;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class Swerve extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private final Supplier<Pose2d> u_PoseSupplier = () -> m_odometry.getEstimatedPosition();
    private final Consumer<Pose2d> u_ResetConsumer = pose -> m_odometry.resetPosition(this.getPigeon2().getRotation2d(), getPositions(), pose);
    private final Supplier<ChassisSpeeds> u_SpeedSupplier = () -> m_kinematics.toChassisSpeeds(getModuleStates());
    private final Consumer<ChassisSpeeds> u_SpeedConsumer = relativeSpeeds -> driveRobotRelative(relativeSpeeds);
    private final HolonomicPathFollowerConfig u_PathFollowerConfig = new HolonomicPathFollowerConfig(new PIDConstants(3,0,0), new PIDConstants(100,0,0.2), 3.0, 5.43023, new ReplanningConfig());

    public Swerve(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {    
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        for (int i = 0; i < 4; i++) {
            SwerveModule s = this.getModule(i);
            s.getDriveMotor().setNeutralMode(NeutralModeValue.Brake); //Changed into Brake mode
            s.getSteerMotor().setNeutralMode(NeutralModeValue.Brake); //Changed into Brake mode
        }
    }
    public Swerve(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        for (int i = 0; i < 4; i++) {
            SwerveModule s = this.getModule(i);
            s.getDriveMotor().setNeutralMode(NeutralModeValue.Brake); //Changed into Brake mode
            s.getSteerMotor().setNeutralMode(NeutralModeValue.Brake); //Changed into Brake mode
        }
        AutoBuilder.configureHolonomic(u_PoseSupplier, u_ResetConsumer, u_SpeedSupplier, u_SpeedConsumer, u_PathFollowerConfig, () -> { var alliance = DriverStation.getAlliance(); if (alliance.isPresent()) { return alliance.get() == DriverStation.Alliance.Red; } return false; }, this);
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

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = this.getModule(i).getCurrentState();
        }
        return states;
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = this.getModule(i).getPosition(false);
        }
        return positions;
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
        SwerveModuleState[] targetStates = m_kinematics.toSwerveModuleStates(targetSpeeds);
        setStates(targetStates);
    }

    public void setStates(SwerveModuleState[] targetStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, 4.5);
        for (int i = 0; i < 4; i++) {
            this.getModule(i).apply(targetStates[i], DriveRequestType.Velocity);
        }
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
