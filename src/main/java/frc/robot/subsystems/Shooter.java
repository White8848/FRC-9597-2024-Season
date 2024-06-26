package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;

public class Shooter extends SubsystemBase {
  private static final String canBusName = "canivore";
  private final TalonFX m_upShooterTalonFX = new TalonFX(21, canBusName);
  private final TalonFX m_downShooterTalonFX = new TalonFX(22, canBusName);
  private final VelocityTorqueCurrentFOC m_torqueVelocity = new VelocityTorqueCurrentFOC(0, 0, 0, 0, false, false,
      false);

  /* What to publish over networktables for telemetry */
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  /* shooter data for checking */
  private final NetworkTable driveStats = inst.getTable("Shooter");
  private final DoublePublisher velocityTarget = driveStats.getDoubleTopic("Velocity Target").publish();
  private final DoublePublisher velocityMeasurement = driveStats.getDoubleTopic("Velocity Measurement").publish();
  private final DoublePublisher torqueTarget = driveStats.getDoubleTopic("Torque Target").publish();
  private final DoublePublisher torqueMeasurement = driveStats.getDoubleTopic("Torque Measurement").publish();
  private final DoublePublisher torqueFeedForward = driveStats.getDoubleTopic("Torque FeedForward").publish();

  public Shooter() {
    initializeTalonFX(m_upShooterTalonFX.getConfigurator());
    initializeTalonFX(m_downShooterTalonFX.getConfigurator());

  }

  public Command commonShootCommand() {
    return startEnd(() -> setVelocity(60.0), () -> setVelocity(0.0));
  }

  public Command commonShootCommand(double velocity, boolean oppositeDirection) {
    if (oppositeDirection == true) {
      return startEnd(() -> setVelocity(-velocity), () -> setVelocity(0.0));
    }
    if (oppositeDirection == false) {
      return startEnd(() -> setVelocity(velocity), () -> setVelocity(0.0));
    }
    return runOnce(() -> setVelocity(0.0));
  }

  public Command differentialShootUpCommand() {
    return startEnd(() -> setVelocity(60, 18.0), () -> setVelocity(0.0)); // 50,20
  }

  public Command farShootCommand() {
    return startEnd(() -> setVelocity(62.0, 50.0), () -> setVelocity(0.0));
  }

  public Command differentialShootDownCommand() {
    return startEnd(() -> setVelocity(75.0, 25.0), () -> setVelocity(0.0));
  }

  public void setVelocity(double velocity) {
    var desiredRotationsPerSecond = velocity;

    if (Math.abs(desiredRotationsPerSecond) <= 1) { // Joystick deadzone
      desiredRotationsPerSecond = 0;
    }
    double friction_torque = (desiredRotationsPerSecond > 0) ? 1 : -1;// To account for friction

    m_upShooterTalonFX.setControl(
        m_torqueVelocity.withVelocity(desiredRotationsPerSecond).withFeedForward(friction_torque));
    m_downShooterTalonFX.setControl(
        m_torqueVelocity.withVelocity(desiredRotationsPerSecond).withFeedForward(friction_torque));
  }

  public void setVelocity(double velocityUp, double velocityDown) {
    var desiredRotationsPerSecond = velocityUp;
    var desiredRotationsPerSecondDown = velocityDown;

    if (Math.abs(desiredRotationsPerSecond) <= 1) { // Joystick deadzone
      desiredRotationsPerSecond = 0;
    }

    m_upShooterTalonFX.setControl(
        m_torqueVelocity.withVelocity(desiredRotationsPerSecond));
    m_downShooterTalonFX.setControl(
        m_torqueVelocity.withVelocity(desiredRotationsPerSecondDown));
  }

  @Override
  public void periodic() {
    velocityTarget.set(m_upShooterTalonFX.getClosedLoopReference().getValueAsDouble());
    velocityMeasurement.set(m_upShooterTalonFX.getVelocity().getValueAsDouble());
    torqueTarget.set(m_upShooterTalonFX.getClosedLoopOutput().getValueAsDouble());
    torqueMeasurement.set(m_upShooterTalonFX.getTorqueCurrent().getValueAsDouble());
    torqueFeedForward.set(m_upShooterTalonFX.getClosedLoopFeedForward().getValueAsDouble());
  }

  private void initializeTalonFX(TalonFXConfigurator cfg) {
    var toApply = new TalonFXConfiguration();

    toApply.Slot0.kP = 6; // An error of 1 rotation per second results in 5 amps output
    toApply.Slot0.kI = 0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
    toApply.Slot0.kD = 0.001; // A change of 1000 rotation per second squared results in 1 amp output

    // Peak output of 40 amps
    toApply.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    toApply.TorqueCurrent.PeakReverseTorqueCurrent = -40;

    cfg.apply(toApply);
  }

  public boolean isShooterOn() {
    if (m_upShooterTalonFX.getClosedLoopReference().getValueAsDouble() != 0.0)
      return true;
    else
      return false;
  }

  public TalonFX getUpShooterTalonFX() {
    return m_upShooterTalonFX;
  }

  public TalonFX getDownShooterTalonFX() {
    return m_downShooterTalonFX;
  }
}
