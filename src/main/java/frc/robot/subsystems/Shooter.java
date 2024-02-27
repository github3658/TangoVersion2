// TODO: Code to calculate motor speed to note distance
// TODO: Finalize shooter controls with drive team

// This is the shooter subsystem.
// Currently, it shoots. We'd like for it to move and pivot to shoot at specific angles with accuracy.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Shooter extends SubsystemBase {
	/* CONSTANTS (prefix: c) */
	private final int c_ShootLeftID   = 14;
	private final int c_ShootRightID  = 15;
	private final int c_ShootPivotID  = 16;
	private final int c_ShootExtendID = 13;

	/* MOTORS (prefix: m) */
	private final TalonFX m_ShootLeft;
	private final TalonFX m_ShootRight;
	private final TalonFX m_ShootPivot;
	private final TalonFX m_ShootExtend;

	/* OTHER VARIABLES */
	private double d_ShooterSpeed = 0.0;

	public Shooter() {
		m_ShootLeft = new TalonFX(c_ShootLeftID, "3658CANivore");
		m_ShootRight = new TalonFX(c_ShootRightID, "3658CANivore");
		m_ShootPivot = new TalonFX(c_ShootPivotID, "3658CANivore");
		m_ShootExtend = new TalonFX(c_ShootExtendID, "3658CANivore");
		m_ShootLeft.getConfigurator().apply(new TalonFXConfiguration());
		m_ShootRight.getConfigurator().apply(new TalonFXConfiguration());
		m_ShootPivot.getConfigurator().apply(new TalonFXConfiguration());
		m_ShootExtend.getConfigurator().apply(new TalonFXConfiguration());

		m_ShootLeft.setNeutralMode(NeutralModeValue.Coast);
		m_ShootLeft.setNeutralMode(NeutralModeValue.Coast);
		m_ShootPivot.setNeutralMode(NeutralModeValue.Brake);
		m_ShootExtend.setNeutralMode(NeutralModeValue.Brake);

		m_ShootLeft.setInverted(true);
		m_ShootRight.setInverted(false);
	}

	@Override
	public void periodic() {
		m_ShootLeft.set(d_ShooterSpeed);
		m_ShootRight.set(d_ShooterSpeed);
		outputTelemetry();
	}

	public void stop() {
		setSpeed(0.0);
	}

	public void outputTelemetry() {
		SmartDashboard.putNumber("Shooter Speed", d_ShooterSpeed);
		SmartDashboard.putNumber("Left Speed", m_ShootLeft.getVelocity().getValueAsDouble());
		SmartDashboard.putNumber("Right Speed", m_ShootRight.getVelocity().getValueAsDouble());
	}

	public void setSpeed(double speed) {
		d_ShooterSpeed = speed;
	}

	public ParentDevice[] requestOrchDevices() {
		ParentDevice[] pd = {m_ShootLeft, m_ShootRight};
		return pd;
	}
}