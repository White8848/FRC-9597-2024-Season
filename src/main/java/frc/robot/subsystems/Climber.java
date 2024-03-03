package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.CoastOut;

public class Climber extends SubsystemBase {
    private final TalonFX m_leftTalonFX = new TalonFX(15);
    private final TalonFX m_rightTalonFX = new TalonFX(16);
    private final PositionTorqueCurrentFOC m_torquePosition = new PositionTorqueCurrentFOC(0, 0, 0, 0, false, false,
            false);
    private final VelocityTorqueCurrentFOC m_torqueVelocity = new VelocityTorqueCurrentFOC(0, 0, 0, 1, true, false,
            false);
    private final NeutralOut m_neutralOut = new NeutralOut();
    private final CoastOut m_coastOut = new CoastOut();

    private final Timer m_timer = new Timer();
    private double m_target = 0.0;

    public Climber() {
        initializeTalonFX(m_leftTalonFX.getConfigurator());
        initializeTalonFX(m_rightTalonFX.getConfigurator());
        m_timer.start();

    }

    public Command leftClimberUp() {
        return startEnd(() -> {
            setVelocity(m_leftTalonFX, 15.0);
            // m_target += 0.1;
            // setPosition(m_target, -m_target);
        }, () -> {
            setBreak(m_leftTalonFX);
        });

    }

    public Command leftClimberDown() {
        return startEnd(() -> {
            setVelocity(m_leftTalonFX, -15.0);
            // m_target -= 0.1;
            // setPosition(m_target, -m_target);
        }, () -> {
            setBreak(m_leftTalonFX);
        });
    }

    public Command rightClimberUp() {
        return startEnd(() -> {
            setVelocity(m_rightTalonFX, 15.0);
            // m_target += 0.1;
            // setPosition(m_target, -m_target);
        }, () -> {
            setBreak(m_rightTalonFX);
        });

    }

    public Command rightClimberDown() {
        return startEnd(() -> {
            setVelocity(m_rightTalonFX, -15.0);
            // m_target -= 0.1;
            // setPosition(m_target, -m_target);
        }, () -> {
            setBreak(m_rightTalonFX);
        });
    }

    public void setVelocity(TalonFX talonFX, double Velocity) {
        talonFX.setControl(m_torqueVelocity.withVelocity(Velocity));
    }

    public void setBreak(TalonFX talonFX) {
        talonFX.setControl(m_neutralOut);
    }

    @Override
    public void periodic() {

    }

    private void initializeTalonFX(TalonFXConfigurator cfg) {
        var toApply = new TalonFXConfiguration();

        toApply.Slot0.kP = 0.0;
        toApply.Slot0.kI = 0.0;
        toApply.Slot0.kD = 0.0;

        toApply.Slot1.kP = 5.0;
        toApply.Slot1.kI = 0.1;
        toApply.Slot1.kD = 0.001;

        // Peak output of 40 amps
        toApply.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        toApply.TorqueCurrent.PeakReverseTorqueCurrent = -40;

        cfg.apply(toApply);
    }

    public TalonFX getLeftTalonFX() {
        return m_leftTalonFX;
    }

    public TalonFX getRightTalonFX() {
        return m_rightTalonFX;
    }
}
