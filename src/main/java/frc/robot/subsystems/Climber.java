// TODO: Finalize climber controls with drive team

// This is the climber subsystem

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
public class Climber extends SubsystemBase {
    /* CONSTANTS (prefix: c) */
    private final int c_ClimbID = 17;
    private final double c_SetClimbSpeed = 0.1;
    private final double c_SetReleaseSpeed = 0.1;

    /* MOTORS (prefix: m) */
    private final TalonFX m_Climb;

    /* OTHER VARIABLES */
    private double d_ClimbSpeed = 0.0;

    public Climber() {
        m_Climb = new TalonFX(c_ClimbID, "3658CANivore");
        m_Climb.setNeutralMode(NeutralModeValue.Brake);
        m_Climb.setInverted(false);
    }

    @Override
    public void periodic() {
        // TODO: Create climber periodic.

        m_Climb.set(d_ClimbSpeed);

        outputTelemetry();
    }

    public void stop() {
        d_ClimbSpeed = 0.0;
    }

    public void outputTelemetry() {
    }

    public void setNeutralMode(NeutralModeValue neutral) {
        m_Climb.setNeutralMode(neutral);
    }

    public void climb() {
        d_ClimbSpeed = c_SetClimbSpeed;
    }

    public void release() {
        d_ClimbSpeed = c_SetReleaseSpeed;
    }
    
    public void stopClimb() {
        d_ClimbSpeed = 0.0;
    }

    // public void tiltLeft() {
    //     d_ClimbLeftSpeed = c_SetReleaseSpeed;
    //     d_ClimbRightSpeed = 0.0;
    // }

    // public void tiltRight() {
    //     d_ClimbLeftSpeed = 0.0;
    //     d_ClimbRightSpeed = c_SetReleaseSpeed;
    // }

    public ParentDevice[] requestOrchDevices() {
        ParentDevice[] pd = {m_Climb};
        return pd;
    }
}
