// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.Commands.VelocityShootCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;

public class RobotContainer implements Sendable {
  private double MaxSpeed = 3; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final Telemetry logger = new Telemetry(MaxSpeed);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  private final Arm m_arm = new Arm();

  private final CommandXboxController m_joystick = new CommandXboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-m_joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                             // negative Y (forward)
            .withVelocityY(-m_joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-m_joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X
                                                                          // (left)
        ));

    new Trigger(
        () -> ((Math.abs(m_joystick.getLeftY()) + Math.abs(m_joystick.getLeftX())
            + Math.abs(m_joystick.getRightX())) < 0.08) ? true : false)
        .whileTrue(drivetrain.applyRequest(() -> brake));

    // reset the field-centric heading on left bumper press
    m_joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    drivetrain.registerTelemetry(logger::telemeterize);

    m_joystick.rightTrigger().whileTrue(m_shooter.commonShootCommand());
    m_joystick.rightBumper().whileTrue(m_shooter.differentialShootDownCommand());
    m_joystick.leftBumper().whileTrue(m_shooter.differentialShootUpCommand());

    m_joystick.x().whileTrue(m_intake.upTake(m_shooter.isShooterOn()));

    m_joystick.a().whileTrue(m_intake.outPut()).whileTrue(m_shooter.commonShootCommand(20.0, true));

    m_joystick.y().whileTrue(m_intake.outPut(5.0)).whileTrue(m_shooter.commonShootCommand(5.0, true));

    // m_joystick.b().whileTrue(m_intake.upTake(15.0)).whileTrue(m_shooter.commonShootCommand());

    m_joystick.b().whileTrue(new VelocityShootCommand(m_intake, m_shooter));

    m_joystick.povUp().onTrue(m_arm.armUp());
    m_joystick.povDown().onTrue(m_arm.armDown());

  }

  @Override
  public void initSendable(SendableBuilder builder) {

  }

}
