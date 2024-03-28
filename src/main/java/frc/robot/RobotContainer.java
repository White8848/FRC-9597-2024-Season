// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import com.pathplanner.lib.auto.AutoBuilder;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.VelocityShootCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.RightClimber;
import frc.robot.subsystems.LeftClimber;
import frc.robot.Constants.DraveTrainConstants;;

public class RobotContainer implements Sendable {
  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final Telemetry logger = new Telemetry(DraveTrainConstants.MaxSpeed);

  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  private final Arm m_arm = new Arm();
  private final RightClimber m_rightClimber = new RightClimber();
  private final LeftClimber m_leftClimber = new LeftClimber();

  private final CommandXboxController m_driverJoystick = new CommandXboxController(0);
  private final CommandXboxController m_operatorJoystick = new CommandXboxController(1);

  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(drivetrain.applyRequest(m_driverJoystick));

    // new Trigger(
    // () -> ((Math.abs(m_joystick.getLeftY()) + Math.abs(m_joystick.getLeftX())
    // + Math.abs(m_joystick.getRightX())) < 0.08) ? true : false)
    // .whileTrue(drivetrain.applyRequest(() -> brake));

    // reset the field-centric heading on left bumper press
    m_driverJoystick.start().onTrue(drivetrain.runOnce(() -> {

      drivetrain.seedFieldRelative();
    }));

    drivetrain.registerTelemetry(logger::telemeterize);

    m_driverJoystick.rightTrigger().whileTrue(m_shooter.commonShootCommand());
    m_driverJoystick.rightBumper().whileTrue(m_shooter.differentialShootDownCommand());
    m_driverJoystick.leftBumper().whileTrue(m_shooter.differentialShootUpCommand());

    m_driverJoystick.x().whileTrue(m_intake.smartUpTake(25.0));

    m_driverJoystick.a().whileTrue(m_intake.outPut()).whileTrue(m_shooter.commonShootCommand(20.0, true));

    m_driverJoystick.y().whileTrue(m_intake.outPut(5.0)).whileTrue(m_shooter.commonShootCommand(5.0, true));

    // m_joystick.b().whileTrue(m_intake.upTake(15.0)).whileTrue(m_shooter.commonShootCommand());

    m_driverJoystick.b().whileTrue(new VelocityShootCommand(m_intake, m_shooter));

    m_driverJoystick.povUp().onTrue(m_arm.armUp());
    m_driverJoystick.povDown().onTrue(m_arm.armDown());

  }

  @Override
  public void initSendable(SendableBuilder builder) {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
