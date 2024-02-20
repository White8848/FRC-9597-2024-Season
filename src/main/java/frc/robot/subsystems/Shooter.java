package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;

public class Shooter extends SubsystemBase {
    private static final String canBusName = "canivore";
    private final TalonFX m_upShooterTalonFX = new TalonFX(6, canBusName);
    private final TalonFX m_downShooterTalonFX = new TalonFX(7, canBusName);

    private final VelocityTorqueCurrentFOC m_torqueVelocity = new VelocityTorqueCurrentFOC(0, 0, 0, 0, false, false,
            false);

    public Shooter() {
        initializeTalonFX(m_upShooterTalonFX.getConfigurator());
        initializeTalonFX(m_downShooterTalonFX.getConfigurator());

    }

    public Command commonShootCommand() {
        return startEnd(
                () -> setVelocity(40.0),
                () -> setVelocity(0.0));

    }

    public Command commonShootCommand(boolean oppositeDirection) {
        if (oppositeDirection == true) {
            return startEnd(
                    () -> setVelocity(-20.0),
                    () -> setVelocity(0.0));
        }
        return runOnce(() -> setVelocity(0.0));

    }

    public Command differentialShootUpCommand() {
        return startEnd(
                () -> setVelocity(45.0, 30.0),
                () -> setVelocity(0.0));
    }

    public Command differentialShootDownCommand() {
        return startEnd(
                () -> setVelocity(60.0, 70.0),
                () -> setVelocity(0.0));
    }

    private void setVelocity(double velocity) {
        var desiredRotationsPerSecond = velocity;

        if (Math.abs(desiredRotationsPerSecond) <= 1) { // Joystick deadzone
            desiredRotationsPerSecond = 0;
        }
        double friction_torque = (desiredRotationsPerSecond > 0) ? 1 : -1;

        m_upShooterTalonFX.setControl(
                m_torqueVelocity.withVelocity(desiredRotationsPerSecond).withFeedForward(friction_torque));
        m_downShooterTalonFX.setControl(
                m_torqueVelocity.withVelocity(desiredRotationsPerSecond).withFeedForward(friction_torque));
    }

    private void setVelocity(double velocityUp, double velocityDown) {
        var desiredRotationsPerSecond = velocityUp;
        var desiredRotationsPerSecondDown = velocityDown;

        if (Math.abs(desiredRotationsPerSecond) <= 1) { // Joystick deadzone
            desiredRotationsPerSecond = 0;
        }
        double friction_torque = (desiredRotationsPerSecond > 0) ? 1 : -1;

        m_upShooterTalonFX.setControl(
                m_torqueVelocity.withVelocity(desiredRotationsPerSecond).withFeedForward(friction_torque));
        m_downShooterTalonFX.setControl(
                m_torqueVelocity.withVelocity(desiredRotationsPerSecondDown).withFeedForward(friction_torque));
    }

    private void initializeTalonFX(TalonFXConfigurator cfg) {
        var toApply = new TalonFXConfiguration();

        toApply.Slot0.kP = 5; // An error of 1 rotation per second results in 5 amps output
        toApply.Slot0.kI = 0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
        toApply.Slot0.kD = 0.001; // A change of 1000 rotation per second squared results in 1 amp output

        // Peak output of 40 amps
        toApply.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        toApply.TorqueCurrent.PeakReverseTorqueCurrent = -40;

        cfg.apply(toApply);
    }

    public TalonFX getUpShooterTalonFX() {
        return m_upShooterTalonFX;
    }

    public TalonFX getDownShooterTalonFX() {
        return m_downShooterTalonFX;
    }
}
