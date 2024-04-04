// Copyright (c) 2024 FRC 9597
// https://github.com/White8848/FRC-9597-2024-Season
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private static final String canBusName = "canivore";
  private final TalonFX m_leftTalonFX = new TalonFX(25, canBusName);
  private final TalonFX m_rightTalonFX = new TalonFX(26, canBusName);
  private final DynamicMotionMagicTorqueCurrentFOC m_mmtorquePosition =
      new DynamicMotionMagicTorqueCurrentFOC(0, 45, 250, 4000, 0, 0, false, false, false);

  private double m_targetArmPosition = 0.0;
  private double m_realArmPosition = 0.0;

  private final double ADD_POSITION = 8;
  private final double MICRO_POSITION = 2;
  private final double PI = 3.1415926;
  private final double MAX_FEEDFORWARD = 5.0;
  private final double MAXIMUM_POSITION = 0.0;
  private final double MINIMUM_POSITION = -38.0;

  /* What to publish over networktables for telemetry */
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  /* intake data for checking */
  private final NetworkTable driveStats = inst.getTable("Arm");
  private final DoublePublisher velocityTarget =
      driveStats.getDoubleTopic("Velocity Target").publish();
  private final DoublePublisher velocityMeasurement =
      driveStats.getDoubleTopic("Velocity Measurement").publish();
  private final DoublePublisher positionTarget =
      driveStats.getDoubleTopic("Position Target").publish();
  private final DoublePublisher positionMeasurement =
      driveStats.getDoubleTopic("Position Measurement").publish();
  private final DoublePublisher torqueTarget = driveStats.getDoubleTopic("Torque Target").publish();
  private final DoublePublisher torqueMeasurement =
      driveStats.getDoubleTopic("Torque Measurement").publish();
  private final DoublePublisher torqueFeedForward =
      driveStats.getDoubleTopic("Torque FeedForward").publish();
  private final DoublePublisher torqueMeasurementRight =
      driveStats.getDoubleTopic("Right Torque Measurement").publish();
  private final DoublePublisher realArmPosition =
      driveStats.getDoubleTopic("Real Arm Position").publish();

  public Arm() {
    initializeTalonFX(m_leftTalonFX.getConfigurator());
    m_leftTalonFX.setNeutralMode(NeutralModeValue.Brake);
    m_rightTalonFX.setNeutralMode(NeutralModeValue.Brake);
    m_rightTalonFX.setControl(new Follower(m_leftTalonFX.getDeviceID(), true));

    m_realArmPosition = -0.003; // -1 degree / 360
  }

  public Command armUp() {
    return runOnce(
        () -> {
          m_mmtorquePosition.Velocity = 35;
          m_mmtorquePosition.Acceleration = 100;
          m_mmtorquePosition.Jerk = 1000;

          var position = getArmPosition() - ADD_POSITION;
          setArmPosition(position);
        });
  }

  public Command armDown() {
    return runOnce(
        () -> {
          m_mmtorquePosition.Velocity = 30;
          m_mmtorquePosition.Acceleration = 100;
          m_mmtorquePosition.Jerk = 1000;

          var position = getArmPosition() + ADD_POSITION;
          setArmPosition(position);
        });
  }

  public Command armUpMicro() {
    return runOnce(
        () -> {
          m_mmtorquePosition.Velocity = 25;
          m_mmtorquePosition.Acceleration = 100;
          m_mmtorquePosition.Jerk = 1000;

          var position = getArmPosition() - MICRO_POSITION;
          setArmPosition(position);
        });
  }

  public Command armDownMicro() {
    return runOnce(
        () -> {
          m_mmtorquePosition.Velocity = 20;
          m_mmtorquePosition.Acceleration = 100;
          m_mmtorquePosition.Jerk = 1000;

          var position = getArmPosition() + MICRO_POSITION;
          setArmPosition(position);
        });
  }

  public Command armBackZero() {
    return runOnce(
        () -> {
          m_mmtorquePosition.Velocity = 60; // 35
          m_mmtorquePosition.Acceleration = 100;
          m_mmtorquePosition.Jerk = 1000;

          var position = 0.0;
          setArmPosition(position);
        });
  }

  public Command armAMP() {
    return runOnce(
        () -> {
          m_mmtorquePosition.Velocity = 50; // 30
          m_mmtorquePosition.Acceleration = 100;
          m_mmtorquePosition.Jerk = 1000;

          var position = -31.4; // -32
          setArmPosition(position);
        });
  }

  public Command armMid() {
    return runOnce(
        () -> {
          m_mmtorquePosition.Velocity = 50;
          m_mmtorquePosition.Acceleration = 100;
          m_mmtorquePosition.Jerk = 1000;

          var position = -7.5; // -6.9
          setArmPosition(position);
        });
  }

  public Command armPod() {
    return runOnce(
        () -> {
          m_mmtorquePosition.Velocity = 50;
          m_mmtorquePosition.Acceleration = 100;
          m_mmtorquePosition.Jerk = 1000;

          var position = -13.0; // -12.8
          setArmPosition(position);
        });
  }

  public void setArmPosition(double position) {
    if (position > MAXIMUM_POSITION) {
      position = MAXIMUM_POSITION;
    } else if (position < MINIMUM_POSITION) {
      position = MINIMUM_POSITION;
    }
    m_targetArmPosition = position;
  }

  private void initializeTalonFX(TalonFXConfigurator cfg) {
    var toApply = new TalonFXConfiguration();

    toApply.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    /*
     * Torque-based velocity does not require a feed forward, as torque will
     * accelerate the rotor up to the desired velocity by itself
     */
    toApply.Slot0.kP = 3.5; // An error of 1 rotation per second results in 5 amps output
    toApply.Slot0.kI =
        0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
    toApply.Slot0.kD = 1.5; // A change of 1000 rotation per second squared results in 1 amp output

    // Peak output of 40 amps
    toApply.TorqueCurrent.PeakForwardTorqueCurrent = 60;
    toApply.TorqueCurrent.PeakReverseTorqueCurrent = -60;

    cfg.apply(toApply);
  }

  @Override
  public void periodic() {
    telemetry();
    m_realArmPosition = -(getArmPosition() / 80.0) * 360 - 5.0;
    var armPositionRad = m_realArmPosition * PI / 180;
    var feedforward = -MAX_FEEDFORWARD * Math.cos(2 * (armPositionRad - 4 / PI));

    m_leftTalonFX.setControl(
        m_mmtorquePosition.withPosition(m_targetArmPosition).withFeedForward(feedforward));

    // Logger.recordOutput("Arm/TargetAngle", m_targetArmPosition);
    // Logger.recordOutput("Arm/CurrentAngle", m_leftTalonFX.getPosition().getValueAsDouble());
  }

  private void telemetry() {
    velocityTarget.set(m_leftTalonFX.getClosedLoopReferenceSlope().getValueAsDouble());
    velocityMeasurement.set(m_leftTalonFX.getVelocity().getValueAsDouble());

    positionTarget.set(m_leftTalonFX.getClosedLoopReference().getValueAsDouble());
    positionMeasurement.set(m_leftTalonFX.getPosition().getValueAsDouble());

    torqueTarget.set(m_leftTalonFX.getClosedLoopOutput().getValueAsDouble());
    torqueMeasurement.set(m_leftTalonFX.getTorqueCurrent().getValueAsDouble());
    torqueFeedForward.set(m_leftTalonFX.getClosedLoopFeedForward().getValueAsDouble());

    torqueMeasurementRight.set(m_rightTalonFX.getTorqueCurrent().getValueAsDouble());

    realArmPosition.set(m_realArmPosition);
  }

  public TalonFX getLeftTalonFX() {
    return m_leftTalonFX;
  }

  public TalonFX getRightTalonFX() {
    return m_rightTalonFX;
  }

  public double getArmPosition() {
    return m_leftTalonFX.getPosition().getValueAsDouble();
  }
}
