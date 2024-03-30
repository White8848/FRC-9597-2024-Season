// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.pathplanner.lib.auto.AutoBuilder;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.VelocityShootCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.RightClimber;
import frc.robot.subsystems.LeftClimber;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.commands.ClimberAutoCommand;

public class RobotContainer implements Sendable {
  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final Telemetry logger = new Telemetry(DriveTrainConstants.MaxSpeed);

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
    drivetrain.setDefaultCommand(drivetrain.applyRequest(m_driverJoystick, m_operatorJoystick));

    // new Trigger(
    // () -> ((Math.abs(m_joystick.getLeftY()) + Math.abs(m_joystick.getLeftX())
    // + Math.abs(m_joystick.getRightX())) < 0.08) ? true : false)
    // .whileTrue(drivetrain.applyRequest(() -> brake));

    // reset the field-centric heading on left bumper press
    m_driverJoystick.start().onTrue(drivetrain.resetFieldRelative());

    drivetrain.registerTelemetry(logger::telemeterize);

    m_driverJoystick.rightTrigger().whileTrue(m_shooter.commonShootCommand());
    m_driverJoystick.rightBumper().whileTrue(m_shooter.differentialShootDownCommand());
    m_driverJoystick.leftBumper().whileTrue(m_shooter.differentialShootUpCommand());

    m_driverJoystick.x().whileTrue(m_intake.smartUpTake(30));

    m_driverJoystick.a().whileTrue(m_intake.outPut()).whileTrue(m_shooter.commonShootCommand(20.0, true));

    m_driverJoystick.b()
        .whileTrue(m_intake.outPut(-35.0))
        .whileTrue(m_shooter.commonShootCommand(40.0, false))
        .onFalse(m_arm.armBackZero());

    // m_joystick.b().whileTrue(m_intake.upTake(15.0)).whileTrue(m_shooter.commonShootCommand());

    m_driverJoystick.y().whileTrue(new VelocityShootCommand(m_intake, m_shooter));

    //////////////////////////// Climber ////////////////////////////
    m_driverJoystick.povRight().whileTrue(m_rightClimber.Down());
    m_driverJoystick.povLeft().whileTrue(m_leftClimber.Down());
    m_driverJoystick.povUp().whileTrue(new ClimberAutoCommand(m_leftClimber, m_rightClimber, "UP"));
    m_driverJoystick.povDown().whileTrue(new ClimberAutoCommand(m_leftClimber, m_rightClimber, "DOWN"));

    m_operatorJoystick.povUp().onTrue(m_arm.armUp());
    m_operatorJoystick.povDown().onTrue(m_arm.armDown());
    m_operatorJoystick.povRight().onTrue(m_arm.armUpMicro());
    m_operatorJoystick.povLeft().onTrue(m_arm.armDownMicro());

    m_operatorJoystick.leftBumper().whileTrue(m_shooter.differentialShootUpCommand());
    m_operatorJoystick.rightBumper().whileTrue(m_shooter.commonShootCommand());
    m_operatorJoystick.leftTrigger().whileTrue(m_shooter.differentialShootDownCommand());
    m_operatorJoystick.rightTrigger().whileTrue(m_shooter.farShootCommand());

    m_operatorJoystick.x().onTrue(m_arm.armBackZero());
    m_operatorJoystick.y().onTrue(m_arm.armAMP());
    // operatorController.a().onTrue(m_arm.armMid());
    // operatorController.b().onTrue(m_arm.armPod());
    m_operatorJoystick.a().whileTrue(m_arm.armMid()).whileTrue(m_shooter.farShootCommand());
    m_operatorJoystick.b().whileTrue(m_arm.armPod()).whileTrue(m_shooter.farShootCommand());

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
