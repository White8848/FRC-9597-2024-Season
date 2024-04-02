package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.CoastOut;

import frc.robot.Constants.Climber;

public class RightClimber extends SubsystemBase {
    private final TalonFX m_TalonFX = new TalonFX(24);
    private final PositionTorqueCurrentFOC m_torquePosition = new PositionTorqueCurrentFOC(0, 0, 0, 0, false, false,
            false);
    private final VelocityTorqueCurrentFOC m_torqueVelocity = new VelocityTorqueCurrentFOC(0, 0, 0, 1, false, false,
            false);

    public RightClimber() {
        initializeTalonFX(m_TalonFX.getConfigurator());
        Climber.RightClimberTarget = m_TalonFX.getPosition().getValueAsDouble();
    }

    public Command Up() {
        return runEnd(
                () -> {
                    setVelocity(40.0);
                    Climber.RightClimberTarget = m_TalonFX.getPosition().getValueAsDouble();
                },

                () -> {
                    setVelocity(0.0);
                    setPosition(Climber.RightClimberTarget);
                });
    }

    public Command Down() {
        return runEnd(
                () -> {
                    setVelocity(-40.0);
                    Climber.RightClimberTarget = m_TalonFX.getPosition().getValueAsDouble();
                },

                () -> {
                    setVelocity(0.0);
                    setPosition(Climber.RightClimberTarget);
                });
    }

    public void setVelocity(double Velocity) {
        m_TalonFX.setControl(m_torqueVelocity.withVelocity(Velocity));
    }

    public void setPosition(double position) {
        m_TalonFX.setControl(m_torquePosition.withPosition(position));
    }

    @Override
    public void periodic() {

    }

    private void initializeTalonFX(TalonFXConfigurator cfg) {
        var toApply = new TalonFXConfiguration();

        toApply.Slot0.kP = 10.0;
        toApply.Slot0.kI = 0.01;
        toApply.Slot0.kD = 0.5;

        toApply.Slot1.kP = 5.0;
        toApply.Slot1.kI = 0.0;
        toApply.Slot1.kD = 0.0;

        // Peak output of 40 amps
        toApply.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        toApply.TorqueCurrent.PeakReverseTorqueCurrent = -80;

        toApply.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        cfg.apply(toApply);
    }

    public TalonFX getTalonFX() {
        return m_TalonFX;
    }
}
