package frc.robot.subsystems;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.units.Per;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.time.Period;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;

public class Intake extends SubsystemBase {
    private static final String canBusName = "canivore";
    private final TalonFX m_intakeTalonFX = new TalonFX(8, canBusName);
    private final VelocityTorqueCurrentFOC m_torqueVelocity = new VelocityTorqueCurrentFOC(0, 0, 0, 0, false, false,
            false);

    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* intake data for checking */
    private final NetworkTable driveStats = inst.getTable("Intake");
    private final DoublePublisher velocityTarget = driveStats.getDoubleTopic("Velocity Target").publish();
    private final DoublePublisher velocityMeasurement = driveStats.getDoubleTopic("Velocity Measurement").publish();
    private final DoublePublisher torqueTarget = driveStats.getDoubleTopic("Torque Target").publish();
    private final DoublePublisher torqueMeasurement = driveStats.getDoubleTopic("Torque Measurement").publish();
    private final DoublePublisher torqueFeedForward = driveStats.getDoubleTopic("Torque FeedForward").publish();

    public Intake() {
        initializeTalonFX(m_intakeTalonFX.getConfigurator());

    }

    public Command upTake() {
        return startEnd(
                () -> setVelocity(-30.0),
                () -> setVelocity(0.0));
    }

    public Command outPut() {
        return startEnd(
                () -> setVelocity(30.0),
                () -> setVelocity(0.0));
    }

    public Command upTake(double velocity) {
        return startEnd(
                () -> setVelocity(-velocity),
                () -> setVelocity(0.0));
    }

    public Command outPut(double velocity) {
        return startEnd(
                () -> setVelocity(velocity),
                () -> setVelocity(0.0));
    }

    public void setVelocity(double desiredRotationsPerSecond) {

        if (Math.abs(desiredRotationsPerSecond) <= 1) { // Joystick deadzone
            desiredRotationsPerSecond = 0;
        }

        m_intakeTalonFX.setControl(m_torqueVelocity.withVelocity(desiredRotationsPerSecond));
    }

    @Override
    public void periodic() {
        velocityTarget.set(m_intakeTalonFX.getClosedLoopReference().getValueAsDouble());
        velocityMeasurement.set(m_intakeTalonFX.getVelocity().getValueAsDouble());
        torqueTarget.set(m_intakeTalonFX.getClosedLoopOutput().getValueAsDouble());
        torqueMeasurement.set(m_intakeTalonFX.getTorqueCurrent().getValueAsDouble());
        torqueFeedForward.set(m_intakeTalonFX.getClosedLoopFeedForward().getValueAsDouble());
    }

    private void initializeTalonFX(TalonFXConfigurator cfg) {
        var toApply = new TalonFXConfiguration();

        /*
         * Torque-based velocity does not require a feed forward, as torque will
         * accelerate the rotor up to the desired velocity by itself
         */
        toApply.Slot0.kP = 10; // An error of 1 rotation per second results in 5 amps output
        toApply.Slot0.kI = 0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
        toApply.Slot0.kD = 0.001; // A change of 1000 rotation per second squared results in 1 amp output

        // Peak output of 40 amps
        toApply.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        toApply.TorqueCurrent.PeakReverseTorqueCurrent = -40;

        cfg.apply(toApply);
    }

    public TalonFX getIntakeTalonFX() {
        return m_intakeTalonFX;
    }

}
