package frc.robot.commands;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class ZeroIntake extends Command {
    private final Intake s_Intake;

    public ZeroIntake(Intake subsystem) {
        s_Intake = subsystem;
        addRequirements(s_Intake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        s_Intake.overridePivotSpeed(0.15);
    }

    @Override
    public void end(boolean interrupted) {
        s_Intake.overridePivotSpeed(0.0);
        //s_Intake.resetOffset();
        s_Intake.setStateToStow();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}