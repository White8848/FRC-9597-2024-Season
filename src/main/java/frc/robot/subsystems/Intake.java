// Copyright (c) 2024 FRC 9597
// https://github.com/White8848/FRC-9597-2024-Season
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;

public class Intake extends SubsystemBase {
  private static final String canBusName = "canivore";
  private final TalonFX m_intakeTalonFX = new TalonFX(20, canBusName);
  private final VelocityTorqueCurrentFOC m_torqueVelocity =
      new VelocityTorqueCurrentFOC(0, 0, 0, 0, false, false, false);
  private LedStrips m_LedStrips = LedStrips.getIns();

  // private final Timer m_infeedTimer;

  private static final double INFEED_TIMER_THRESHOLD = 1.0;

  /* What to publish over networktables for telemetry */
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  /* intake data for checking */
  private final NetworkTable driveStats = inst.getTable("Intake");
  private final DoublePublisher velocityTarget =
      driveStats.getDoubleTopic("Velocity Target").publish();
  private final DoublePublisher velocityMeasurement =
      driveStats.getDoubleTopic("Velocity Measurement").publish();
  private final DoublePublisher torqueTarget = driveStats.getDoubleTopic("Torque Target").publish();
  private final DoublePublisher torqueMeasurement =
      driveStats.getDoubleTopic("Torque Measurement").publish();
  private final DoublePublisher torqueFeedForward =
      driveStats.getDoubleTopic("Torque FeedForward").publish();

  private final DigitalInput lighterTrigger = new DigitalInput(0);
  private final Shooter m_shooter = new Shooter();
  private boolean m_stop = false;
  private final Timer m_timer = new Timer();

  public Intake() {
    initializeTalonFX(m_intakeTalonFX.getConfigurator());
    m_timer.start();
    // m_infeedTimer = new Timer();
    // m_infeedTimer.stop();
    // m_infeedTimer.reset();
  }

  public Command upTake() {
    return startEnd(() -> setVelocity(-40.0), () -> setVelocity(0.0));
  }

  public Command upTake(double velocity) {
    return startEnd(() -> setVelocity(-velocity), () -> setVelocity(0.0));
  }

  public BooleanSupplier seesNote() {
    return lighterTrigger::get;
  }

  public Command smartUpTake(double velocity) {
    return runEnd(
        () -> {
          SmartDashboard.putNumber("Lighter Trigger Time", m_timer.get());
          if (lighterTrigger.get() && m_shooter.isShooterOn() == false && m_stop == false) {
            m_stop = true;
            m_timer.reset();
          }
          if (m_stop == true && m_shooter.isShooterOn() == false) {
            if (m_timer.get() < 0.1) {
              setVelocity(0.0);
              m_LedStrips.setRGB(0, 15, 255);
            } else if (m_timer.get() < 0.38) { // 0.30
              setVelocity(4);
              m_LedStrips.setRGB(255, 0, 220);
            }
            // else if (m_timer.get() < 1.5){
            // m_LedStrips.setRGB(0, 255, 0);
            // setVelocity(0.0);
            // }
            else setVelocity(0.0);
            m_LedStrips.setRGB(0, 255, 0);
          } else {
            setVelocity(-velocity);
            m_stop = false;
          }
        },
        () -> setVelocity(0.0));
  }

  public Command outPut() {
    return startEnd(
        () -> {
          setVelocity(45.0);
          m_stop = false;
        },
        () -> setVelocity(0.0));
  }

  public Command outPut(double velocity) {
    return startEnd(
        () -> {
          setVelocity(velocity);
          m_stop = false;
        },
        () -> setVelocity(0.0));
  }

  public Command stop() {
    return runOnce(() -> setVelocity(0.0));
  }

  public void setVelocity(double desiredRotationsPerSecond) {

    double friction_torque = (desiredRotationsPerSecond > 0) ? 1 : -1; // To account for friction
    m_intakeTalonFX.setControl(
        m_torqueVelocity.withVelocity(desiredRotationsPerSecond).withFeedForward(friction_torque));
  }

  private void initializeTalonFX(TalonFXConfigurator cfg) {
    var toApply = new TalonFXConfiguration();

    /*
     * Torque-based velocity does not require a feed forward, as torque will
     * accelerate the rotor up to the desired velocity by itself
     */
    toApply.Slot0.kP = 12; // An error of 1 rotation per second results in 5 amps output
    toApply.Slot0.kI =
        0.2; // An error of 1 rotation per second increases output by 0.1 amps every second
    toApply.Slot0.kD =
        0.001; // A change of 1000 rotation per second squared results in 1 amp output

    // Peak output of 40 amps
    toApply.TorqueCurrent.PeakForwardTorqueCurrent = 60;
    toApply.TorqueCurrent.PeakReverseTorqueCurrent = -60;

    cfg.apply(toApply);
  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("BeamBreak", m_stop);

    velocityTarget.set(m_intakeTalonFX.getClosedLoopReference().getValueAsDouble());
    velocityMeasurement.set(m_intakeTalonFX.getVelocity().getValueAsDouble());
    torqueTarget.set(m_intakeTalonFX.getClosedLoopOutput().getValueAsDouble());
    torqueMeasurement.set(m_intakeTalonFX.getTorqueCurrent().getValueAsDouble());
    torqueFeedForward.set(m_intakeTalonFX.getClosedLoopFeedForward().getValueAsDouble());
  }

  public TalonFX getIntakeTalonFX() {
    return m_intakeTalonFX;
  }
}
