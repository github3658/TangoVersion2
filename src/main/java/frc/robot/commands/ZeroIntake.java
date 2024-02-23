package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class ZeroIntake extends Command {
    private final Intake s_Intake;
    private double d_LastValue = 0.0;

    public ZeroIntake(Intake subsystem) {
        s_Intake = subsystem;
        addRequirements(s_Intake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        if (s_Intake.getPivotCurrent() > 10.0) {
            end(true);
        }
        s_Intake.overridePivotSpeed(0.05);
        d_LastValue = s_Intake.getPivotAngle();
    }

    @Override
    public void end(boolean interrupted) {
        s_Intake.resetOffset();
        s_Intake.setStateToStow();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}