// This command is the default Intake command.
// It accepts operator input and modifies the Intake state from that.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Intake.PivotTarget;

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
        if (xb_Operator.getRawButton(XboxController.Button.kLeftBumper.value)) {
            // Up - Ground State
            if (xb_Operator.getPOV() == 0 && s_Intake.getPivotTarget() != PivotTarget.Ground) {
                s_Intake.setStateToGround();
            }
            // Right - Amp State
            else if (xb_Operator.getPOV() == 90 && s_Intake.getPivotTarget() != PivotTarget.Amp) {
                s_Intake.setStateToAmp();
            }
            // Down - Stow State
            else if (xb_Operator.getPOV() == 180 && s_Intake.getPivotTarget() != PivotTarget.Stow) {
                s_Intake.setStateToStow();
            }

            // A - Force Intake
            if (xb_Operator.getRawButtonPressed(XboxController.Button.kA.value)) {
                s_Intake.setIntake(IntakeState.Intake);
            }
            // B - Force Eject
            if (xb_Operator.getRawButtonPressed(XboxController.Button.kB.value)) {
                s_Intake.setIntake(IntakeState.FastEject);
            }
            // Y - Force Stop
            if (xb_Operator.getRawButtonPressed(XboxController.Button.kY.value)) {
                s_Intake.setIntake(IntakeState.None);
            }
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