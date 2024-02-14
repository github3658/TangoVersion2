// TODO: Find and define limits for shooter motors?
// TODO: Code to calculate motor speed to note distance

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import com.ctre.phoenix6.controls.DutyCycleOut;
//import com.ctre.phoenix6.controls.PositionVoltage;
//import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class Shooter extends SubsystemBase {
    /* CONSTANTS (prefix: c) */
    private final int c_ShootLeftID = 99; // TODO: Define values for shooter motor IDs
    private final int c_ShootRightID = 99;

    private final TalonFX m_ShootLeft;
    private final TalonFX m_ShootRight;

    private Shooter() {
        m_ShootLeft = new TalonFX(c_ShootLeftID);
        m_ShootRight = new TalonFX(c_ShootRightID);
    }
}