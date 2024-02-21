package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

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
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}