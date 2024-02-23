package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;

public class IntakeTeleop extends Command {
    private final Intake s_Intake;

    private GenericHID xb_Operator;

    private final int ctrl_Operator_Intake = XboxController.Button.kA.value;

    public IntakeTeleop(Intake subsystem, GenericHID port1) {
        s_Intake = subsystem;
        xb_Operator = port1;
        addRequirements(s_Intake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (xb_Operator.getRawButtonPressed(ctrl_Operator_Intake)) {
            s_Intake.setStateToGround();
        }
        if (xb_Operator.getRawButtonPressed(XboxController.Button.kY.value)) {
            s_Intake.setStateToStow();
        }

        // if (xb_Operator.getRawButton(XboxController.Button.kRightBumper.value)) {
        //     double left = xb_Operator.getRawAxis(XboxController.Axis.kLeftTrigger.value);
        //     double right = xb_Operator.getRawAxis(XboxController.Axis.kRightTrigger.value);
        //     s_Intake.overridePivotSpeed((right - left)*0.35);
        // }
        // else {
        //     s_Intake.overridePivotSpeed(0.0);
        // }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}