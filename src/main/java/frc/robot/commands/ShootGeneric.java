// This command uses the intake and the shooter to expel notes at a given speed.
// While this command is running, the intake and the shooter are not controllable.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;

public class ShootGeneric extends Command {
    private final Shooter s_Shooter;
    private final Intake s_Intake;

    private int i_ShooterWarmupDelay;
    private int i_ShutdownDelay;
    private boolean b_SeenNote;
    private double d_ShooterSpeed = 0.0;

    public ShootGeneric(Shooter shooter, Intake intake, double speed) {
        s_Shooter = shooter;
        s_Intake = intake;
        d_ShooterSpeed = speed;
        addRequirements(s_Shooter, s_Intake);
    }

    @Override
    public void initialize() {
        i_ShooterWarmupDelay = 50;
        i_ShutdownDelay = 50;
        b_SeenNote = false;
    }

    @Override
    public void execute() {
        s_Shooter.setSpeed(d_ShooterSpeed);
        if (i_ShooterWarmupDelay > 0) {
            i_ShooterWarmupDelay--;
        }
        else {
            s_Intake.eject();
            b_SeenNote = true;
        }

        if (b_SeenNote) {
            i_ShutdownDelay--;
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Finished");
        s_Shooter.setSpeed(0.0);
        s_Intake.setStateToStow();
    }

    @Override
    public boolean isFinished() {
        System.out.println(i_ShutdownDelay);
        return i_ShutdownDelay < 0;
    }
}