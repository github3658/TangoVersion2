// TODO: Define limit for intake tilt?

package frc.robot.subsystems;
//import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
//import com.ctre.phoenix6.controls.DutyCycleOut;
//import com.ctre.phoenix6.controls.PositionVoltage;
//import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
//import frc.robot.Helpers;

public class Intake extends SubsystemBase {
    /* CONSTANTS (prefix: c) */
    private final int c_IntakeNoteID = 10;
    private final int c_IntakePivotID = 9;
    private final double c_EncoderOffset = 0; // TODO: Define intake encoder offset

    /* ENUMS */
    public enum PivotTarget {
        None,
        Ground,
        Source,
        Amp,
        Stow
    }

    public enum IntakeState {
        None,
        Intake,
        Eject,
        Pulse,
        FeedShooter,
    }

    /* MOTORS (prefix: m) */
    private final TalonFX m_IntakeNote;
    private final TalonFX m_IntakePivot;

    /* OTHER VARIABLES */
    private double d_IntakePivotVoltage = 0.0;
    private double d_IntakeSpeed       = 0.0;
    private boolean b_noteLoaded = false;
    private PivotTarget e_PivotTarget  = PivotTarget.Stow;
    private IntakeState e_IntakeState  = IntakeState.None;

    public Intake() {
        m_IntakeNote = new TalonFX(c_IntakeNoteID);
        m_IntakeNote.getConfigurator().apply(new TalonFXConfiguration());
        m_IntakeNote.setNeutralMode(NeutralModeValue.Coast);

        m_IntakePivot = new TalonFX(c_IntakePivotID);
        m_IntakePivot.getConfigurator().apply(new TalonFXConfiguration());
        m_IntakePivot.setNeutralMode(NeutralModeValue.Brake);
        // Limit?
    }

    @Override
    public void periodic() {
        checkAutoTasks();

        // Pivot Control
        double d_PivotAngle = pivotTargetToAngle(e_PivotTarget);
        // TODO: Intake periodic: Find equivalent for PID calculate method
        //d_IntakePivotVoltage = m_pivotPID.calculate(getPivotAngleDegrees(), pivot_angle);

        // Intake Control
        d_IntakeSpeed = intakeStateToSpeed(e_IntakeState);

        // Stow on detect
        if (intakeHasNote() && !b_noteLoaded) {
            e_PivotTarget = PivotTarget.Stow;
            e_IntakeState = IntakeState.None;
            b_noteLoaded = true;
        }
        else if (!intakeHasNote() && b_noteLoaded) {
            b_noteLoaded = false;
        }

        outputTelemetry();
    }

    public void stop() {
        d_IntakePivotVoltage = 0.0;
        d_IntakeSpeed = 0.0;
    }

    public void outputTelemetry() {
        SmartDashboard.putNumber("Intake Speed", intakeStateToSpeed(e_IntakeState));
        SmartDashboard.putNumber("Pivot Encoder (get)", m_IntakePivot.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Pivot Encoder (angle)", getPivotAngleDegrees());
        SmartDashboard.putNumber("Pivot Target Angle", pivotTargetToAngle(e_PivotTarget));

        SmartDashboard.putBoolean("Has Note?", intakeHasNote());
    }

    public double pivotTargetToAngle(PivotTarget target) {
        switch (target) {
            case Ground:
            case Source:
            case Amp:
            case Stow:
            default:
                return 180;
        }
    }

    public double intakeStateToSpeed(IntakeState state) {
        switch (state) {
            case Intake:
            case Eject:
            case Pulse:
            case FeedShooter:
            default:
                return 0.0;
        }
    }

    // Public functions, so commands and subsystems can get info about the intake
    public IntakeState getIntakeState() {
        return e_IntakeState;
    }

    // TODO: getPivotAngleDegrees: Import Helpers or find modRotations equivalent
    public double getPivotAngleDegrees() {
        return 0;
        //return Units.rotationsToDegrees(Helpers.modRotations(m_IntakePivot.getPosition().getValueAsDouble() - c_EncoderOffset + 0.5));
    }

    public boolean intakeHasNote() {
        // limit switch or detection method of some kind.
        // TODO: Implement intakeHasNote
        return false;
    }

    // Pivot functions
    public void setStateToGround() {
        e_PivotTarget = PivotTarget.Ground;
        e_IntakeState = IntakeState.Intake;
    }

    public void setStateToSource() {
        e_PivotTarget = PivotTarget.Source;
        e_IntakeState = IntakeState.None;
    }

    public void setStateToAmp() {
        e_PivotTarget = PivotTarget.Amp;
        e_IntakeState = IntakeState.None;
    }

    public void setStatetoStow() {
        e_PivotTarget = PivotTarget.Stow;
        e_IntakeState = IntakeState.None;
    }

    public void setPivot(PivotTarget target) {
        e_PivotTarget = target;
    }

    // Intake functions
    public void intake() {
        e_IntakeState = IntakeState.Intake;
    }

    public void eject() {
        e_IntakeState = IntakeState.Eject;
    }

    public void pulse() {
        e_IntakeState = IntakeState.Pulse;
    }

    public void feedShooter() {
        e_IntakeState = IntakeState.FeedShooter;
    }

    public void stopIntake() {
        e_IntakeState = IntakeState.None;
        d_IntakeSpeed = 0.0;
    }

    public void setIntake(IntakeState state) {
        e_IntakeState = state;
    }

    // Private functions
    private void checkAutoTasks() {
        if (e_PivotTarget == PivotTarget.Ground && intakeHasNote() && isPivotAtTarget()) {
            e_PivotTarget = PivotTarget.Stow;
            e_IntakeState = IntakeState.None;
        }
    }

    private boolean isPivotAtTarget() {
        return Math.abs(getPivotAngleDegrees() - pivotTargetToAngle(e_PivotTarget)) < 5;
    }

    public ParentDevice[] requestOrchDevices() {
        ParentDevice[] pd = {m_IntakeNote, m_IntakePivot};
        return pd;
    }
}