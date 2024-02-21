// TODO: Define limits for lift motor?
// TODO: Finalize climber controls with drive team


package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
public class Climber extends SubsystemBase {
    /* CONSTANTS (prefix: c) */
    private final int c_ClimbLeftID = 99; // TODO: Find climber motor IDs.
    private final int c_ClimbRightID = 99;
    private final double c_SetClimbSpeed = 0.0; // TODO: The climb/release speeds are 600/-600 RPM respectively. Can TalonFX use RPM values for input?
    private final double c_SetReleaseSpeed = 0.0;

    /* MOTORS (prefix: m) */
    private final TalonFX m_ClimbLeft;
    private final TalonFX m_ClimbRight;

    /* OTHER VARIABLES */
    private double d_ClimbLeftSpeed = 0.0;
    private double d_ClimbRightSpeed = 0.0;

    public Climber() {
        m_ClimbLeft = new TalonFX(c_ClimbLeftID, "3658CANivore");
        m_ClimbRight = new TalonFX(c_ClimbRightID, "3658CANivore");

        // TODO: Configure encoder ratios?

        m_ClimbLeft.setNeutralMode(NeutralModeValue.Brake);
        m_ClimbRight.setNeutralMode(NeutralModeValue.Brake);

        m_ClimbLeft.setInverted(false);
        m_ClimbRight.setInverted(true);
    }

    @Override
    public void periodic() {
        // TODO: Create climber periodic.
        outputTelemetry();
    }

    public void stop() {
        d_ClimbLeftSpeed = 0.0;
        d_ClimbRightSpeed = 0.0;
    }

    public void outputTelemetry() {
        SmartDashboard.putNumber("Left Target Speed", d_ClimbLeftSpeed);
        SmartDashboard.putNumber("Left Real Speed",m_ClimbLeft.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Right Target Speed", d_ClimbRightSpeed);
        SmartDashboard.putNumber("Right Real Speed",m_ClimbRight.getVelocity().getValueAsDouble());
    }

    public void setNeutralMode(NeutralModeValue neutral) {
        m_ClimbLeft.setNeutralMode(neutral);
        m_ClimbRight.setNeutralMode(neutral);
    }

    public void climb() {
        d_ClimbLeftSpeed = c_SetClimbSpeed;
        d_ClimbRightSpeed = c_SetClimbSpeed;
    }

    public void release() {
        d_ClimbLeftSpeed = c_SetReleaseSpeed;
        d_ClimbRightSpeed = c_SetReleaseSpeed;
    }

    public void tiltLeft() {
        d_ClimbLeftSpeed = c_SetReleaseSpeed;
        d_ClimbRightSpeed = 0.0;
    }

    public void tiltRight() {
        d_ClimbLeftSpeed = 0.0;
        d_ClimbRightSpeed = c_SetReleaseSpeed;
    }

    public ParentDevice[] requestOrchDevices() {
        ParentDevice[] pd = {m_ClimbLeft, m_ClimbRight};
        return pd;
    }
}
