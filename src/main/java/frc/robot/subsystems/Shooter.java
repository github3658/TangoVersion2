// TODO: Find and define limits for shooter motors?
// TODO: Code to calculate motor speed to note distance

package frc.robot.subsystems;
//import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
//import com.ctre.phoenix6.controls.DutyCycleOut;
//import com.ctre.phoenix6.controls.PositionVoltage;
//import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Shooter extends SubsystemBase {
    /* CONSTANTS (prefix: c) */
    private final int c_ShootLeftID = 99; // TODO: Define values for shooter motor IDs
    private final int c_ShootRightID = 99;

    /* MOTORS (prefix: m) */
    private final TalonFX m_ShootLeft;
    private final TalonFX m_ShootRight;

    /* OTHER VARIABLES */
    private double d_ShooterRPM = 0.0;

    public Shooter() {
        m_ShootLeft = new TalonFX(c_ShootLeftID);
        m_ShootRight = new TalonFX(c_ShootRightID);
        m_ShootLeft.getConfigurator().apply(new TalonFXConfiguration());
        m_ShootRight.getConfigurator().apply(new TalonFXConfiguration());

        m_ShootLeft.setNeutralMode(NeutralModeValue.Coast);
        m_ShootLeft.setNeutralMode(NeutralModeValue.Coast);

        m_ShootLeft.setInverted(true);
        m_ShootRight.setInverted(false);
    }

    @Override
    public void periodic() {
        outputTelemetry();
    }

    public void stop() {
        setSpeed(0.0);
    }

    public void outputTelemetry() {
        SmartDashboard.putNumber("Shooter RPM", d_ShooterRPM);
        SmartDashboard.putNumber("Left Speed", m_ShootLeft.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Right Speed", m_ShootRight.getVelocity().getValueAsDouble());
    }

    public void setSpeed(double rpm) {
        d_ShooterRPM = rpm;
    }

    public ParentDevice[] requestOrchDevices() {
        ParentDevice[] pd = {m_ShootLeft, m_ShootRight};
        return pd;
    }
}