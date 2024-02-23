package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterTeleop extends Command {
    private final Shooter s_Shooter;

    private GenericHID xb_Operator;

    private final int ctrl_Operator_ButtonB = XboxController.Button.kB.value;

    public ShooterTeleop(Shooter subsystem, GenericHID port1) {
        s_Shooter = subsystem;
        xb_Operator = port1;
        addRequirements(s_Shooter);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // if (xb_Operator.getRawButtonPressed(ctrl_Operator_ButtonB)) {
        //     s_Shooter.setSpeed(0.6);
        // }
        // if (xb_Operator.getRawButtonPressed(XboxController.Button.kStart.value)) {
        //     s_Shooter.setSpeed(0.0);
        // }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}