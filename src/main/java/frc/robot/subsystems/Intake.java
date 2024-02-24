// TODO: Define limit for intake tilt?
// TODO: Finalize intake controls with drive team

package frc.robot.subsystems;
//import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
//import com.ctre.phoenix6.controls.DutyCycleOut;
//import com.ctre.phoenix6.controls.PositionVoltage;
//import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
//import frc.robot.Helpers;
import edu.wpi.first.wpilibj.DigitalInput;

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

    /* SENSORS (prefix: n) */
    private final DutyCycleEncoder n_Encoder;
    private final DigitalInput n_NoteDetect;

    /* OTHER VARIABLES */
    private double d_IntakeSpeed = 0.0;
    private double d_IntakePivotSpeed = 0.0;
    private double d_PivotOffset = 0.0;
    private int i_IntakeSwitchDelay = 0;
    private PivotTarget e_PivotTarget = PivotTarget.Stow;
    private IntakeState e_IntakeState = IntakeState.None;
    private IntakeState e_IntakeStateGOAL = IntakeState.None;

    public Intake() {
        m_IntakeNote = new TalonFX(c_IntakeNoteID, "3658CANivore");
        m_IntakeNote.getConfigurator().apply(new TalonFXConfiguration());
        m_IntakeNote.setNeutralMode(NeutralModeValue.Coast);

        m_IntakePivot = new TalonFX(c_IntakePivotID, "3658CANivore");
        m_IntakePivot.getConfigurator().apply(new TalonFXConfiguration());
        m_IntakePivot.setNeutralMode(NeutralModeValue.Brake);
        
        d_PivotOffset = m_IntakePivot.getPosition().getValueAsDouble();

        n_Encoder = new DutyCycleEncoder(8);
        n_NoteDetect = new DigitalInput(9);

        // TODO: How do you reference a through bore encoder without SparkMAX? Is it just an analogue input?
    }

    @Override
    public void periodic() {
        checkAutoTasks();

        // Pivot Control
        double d_PivotAngle = pivotTargetToAngle(e_PivotTarget);
        // TODO: Intake periodic: Find equivalent for PID calculate method. This is probably just math with the encoder value and motor speed. As it stands, we cannot do this until we find a way to reference the encoder.

        // Intake Control
        if (i_IntakeSwitchDelay > 0) {  // Sometimes it is necessary to delay the intake state for power management or to ensure we have a note
            i_IntakeSwitchDelay--;
        }
        else {
            e_IntakeState = e_IntakeStateGOAL;
        }

        d_IntakeSpeed = intakeStateToSpeed(e_IntakeState);
        if (isPivotAtTarget() || i_IntakeSwitchDelay > 0) {
            m_IntakeNote.set(d_IntakeSpeed);
        }
        else {
            m_IntakeNote.set(0.0);
        }
       

        // Pivot control
        if (e_PivotTarget != PivotTarget.None) {
            double d_CurrentPivot = getPivotAngle();
            d_IntakePivotSpeed = Math.max(Math.min(((d_PivotAngle - d_CurrentPivot) / 20 * 0.35),0.40),-0.40);
        }
        m_IntakePivot.set(d_IntakePivotSpeed);

        // Stow on detect ground note
        if (e_PivotTarget == PivotTarget.Ground && intakeHasNote()) {
            //i_IntakeSwitchDelay = 12;
            setStateToStow();
        }

        outputTelemetry();
    }

    public void stop() {
        d_IntakeSpeed = 0.0;
    }

    public void outputTelemetry() {
        //SmartDashboard.putNumber("Intake Speed", intakeStateToSpeed(e_IntakeState));
        SmartDashboard.putNumber("Pivot Encoder", getPivotAngle());
        SmartDashboard.putNumber("Pivot Target Angle", pivotTargetToAngle(e_PivotTarget));
        SmartDashboard.putBoolean("Has Note?", intakeHasNote());
        SmartDashboard.putBoolean("Pivot in place?", isPivotAtTarget());
        SmartDashboard.putNumber("Pivot Stator Current", getPivotCurrent());
        //SmartDashboard.putNumber("Pivot Speed", d_IntakePivotSpeed);
    }

    public double pivotTargetToAngle(PivotTarget target) {
        switch (target) {
            case Ground:
                return -42.0;
            case Source:
            case Amp:
            case Stow:
            default:
                return 0.0;
        }
    }

    public double intakeStateToSpeed(IntakeState state) {
        switch (state) {
            case Intake:
                return 0.35;
            case Eject:
                return -0.30;
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

    public double getPivotAngle() {
        return n_Encoder.get();
    }

    public boolean intakeHasNote() {
        return !n_NoteDetect.get();
    }

    public void resetOffset() {
        n_Encoder.reset();
    }

    public double getPivotCurrent() {
        return m_IntakePivot.getStatorCurrent().getValueAsDouble();
    }

    // Pivot functions
    public void setStateToGround() {
        e_PivotTarget = PivotTarget.Ground;
        e_IntakeStateGOAL = IntakeState.Intake;
    }

    public void setStateToSource() {
        e_PivotTarget = PivotTarget.Source;
        e_IntakeStateGOAL = IntakeState.None;
    }

    public void setStateToAmp() {
        e_PivotTarget = PivotTarget.Amp;
        e_IntakeStateGOAL = IntakeState.None;
    }

    public void setStateToStow() {
        e_PivotTarget = PivotTarget.Stow;
        e_IntakeStateGOAL = IntakeState.None;
    }

    public void setPivot(PivotTarget target) {
        e_PivotTarget = target;
    }

    public void overridePivotSpeed(double speed) {
        e_PivotTarget = PivotTarget.None;
        d_IntakePivotSpeed = speed;
    }

    // Intake functions
    public void intake() {
        e_IntakeStateGOAL = IntakeState.Intake;
        i_IntakeSwitchDelay = 0;
    }

    public void eject() {
        e_IntakeStateGOAL = IntakeState.Eject;
        i_IntakeSwitchDelay = 0;
    }

    public void pulse() {
        e_IntakeStateGOAL = IntakeState.Pulse;
        i_IntakeSwitchDelay = 0;
    }

    public void feedShooter() {
        e_IntakeStateGOAL = IntakeState.FeedShooter;
        i_IntakeSwitchDelay = 0;
    }

    public void stopIntake() {
        e_IntakeStateGOAL = IntakeState.None;
        d_IntakeSpeed = 0.0;
        i_IntakeSwitchDelay = 0;
    }

    public void setIntake(IntakeState state) {
        e_IntakeStateGOAL = state;
        i_IntakeSwitchDelay = 0;
    }

    // Private functions
    private void checkAutoTasks() {
        // if (e_PivotTarget == PivotTarget.Ground && intakeHasNote() && isPivotAtTarget()) {
        //     e_PivotTarget = PivotTarget.Stow;
        //     e_IntakeState = IntakeState.None;
        // }
    }

    private boolean isPivotAtTarget() {
        return Math.abs(getPivotAngle() - pivotTargetToAngle(e_PivotTarget)) < 5;
    }

    public ParentDevice[] requestOrchDevices() {
        ParentDevice[] pd = {m_IntakeNote, m_IntakePivot};
        return pd;
    }
}