package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.CoastOut;

public class LeftClimber extends SubsystemBase {
    private final TalonFX m_TalonFX = new TalonFX(23);
    private final PositionTorqueCurrentFOC m_torquePosition = new PositionTorqueCurrentFOC(0, 0, 0, 0, false, false,
            false);
    private final VelocityTorqueCurrentFOC m_torqueVelocity = new VelocityTorqueCurrentFOC(0, 0, 0, 1, false, false,
            false);
    private final NeutralOut m_neutralOut = new NeutralOut();
    private final CoastOut m_coastOut = new CoastOut();

    private final Timer m_timer = new Timer();

    public LeftClimber() {
        initializeTalonFX(m_TalonFX.getConfigurator());
        m_timer.start();
    }

    public Command Up(){
        return runEnd(
            () -> setVelocity(0.0),
            () -> setVelocity(0.0)
        );
    }

    private void setVelocity(double Velocity) {
        m_TalonFX.setControl(m_torqueVelocity.withVelocity(Velocity));
    }

    private void setBreak(TalonFX talonFX) {
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

        toApply.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        cfg.apply(toApply);
    }

    public TalonFX getTalonFX() {
        return m_TalonFX;
    }
}
