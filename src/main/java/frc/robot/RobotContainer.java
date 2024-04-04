// Copyright (c) 2024 FRC 9597
// https://github.com/White8848/FRC-9597-2024-Season
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveTrainConstants;
// import frc.robot.commands.ClimberAutoCommand;
import frc.robot.commands.PositionClimbLeftPID;
import frc.robot.commands.PositionClimbRightPID;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ClimberLeft;
import frc.robot.subsystems.ClimberRight;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LedStrips;
// import frc.robot.subsystems.LeftClimber;
// import frc.robot.subsystems.RightClimber;
import frc.robot.subsystems.Shooter;

public class RobotContainer implements Sendable {
  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final ClimberLeft climberleft = new ClimberLeft();
  private final ClimberRight climberright = new ClimberRight();

  private LedStrips m_LedStrips = LedStrips.getIns();
  private final Telemetry logger = new Telemetry(DriveTrainConstants.MaxSpeed);

  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  private final Arm m_arm = new Arm();
  // private final RightClimber m_rightClimber = new RightClimber();
  // private final LeftClimber m_leftClimber = new LeftClimber();

  private final CommandXboxController m_driverJoystick = new CommandXboxController(0);
  private final CommandXboxController m_operatorJoystick = new CommandXboxController(1);

  private final SendableChooser<Command> autoChooser;

  private final Trigger m_seesNote;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_seesNote = new Trigger(m_intake.seesNote());

    // Configure the Named Commands

    // NamedCommands.registerCommand("Intake",
    // m_intake.smartUpTakeTimer(30).withTimeout(4));

    NamedCommands.registerCommand("Intake", m_intake.upTake(30).repeatedly().withTimeout(3));

    NamedCommands.registerCommand(
        "Fix Intake",
        m_intake
            .outPut(4)
            .repeatedly()
            .withTimeout(0.325)
            .andThen(m_intake.runOnce(() -> m_intake.setVelocity(0))));
    // NamedCommands.registerCommand("Shoot", m_shooter.farShootCommand());
    // NamedCommands.registerCommand("Shoot",
    // Commands.run(() -> m_intake.setVelocity(30))
    // .withTimeout(1)
    // .andThen(Commands.run(() -> m_intake.setVelocity(0))));

    // NamedCommands.registerCommand(
    // "Fix Infeed",
    // m_intake
    // .run(() -> m_intake.setVelocity(-15))
    // .alongWith(m_shooter.run(() -> m_shooter.setVelocity(-15)))
    // .withTimeout(0.28)
    // .andThen(m_intake.runOnce(() -> m_intake.setVelocity(0)))); // TODO: 没用。我算了:(
    // yong :(

    NamedCommands.registerCommand(
        "Shoot",
        Commands.parallel(
                m_shooter.differentialShootUpCommand(),
                Commands.waitSeconds(0.6)
                    .andThen(
                        Commands.run(() -> m_intake.setVelocity(-50))
                            // .andThen(Commands.waitSeconds(1))
                            .withTimeout(0.8)
                            .andThen(Commands.runOnce(() -> m_intake.setVelocity(0)))))
            .withTimeout(1.4));

    NamedCommands.registerCommand("Reset Heading", drivetrain.resetFieldRelative());
    // Configure the trigger bindings
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public void stopThings() {
    m_shooter.setVelocity(0.0);
    m_intake.setVelocity(0.0);
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

    m_driverJoystick.leftTrigger().whileTrue(m_shooter.farShootCommand());
    m_driverJoystick.leftBumper().whileTrue(m_shooter.differentialShootUpCommand());
    m_driverJoystick
        .rightBumper()
        .whileTrue(m_intake.outPut(5))
        .whileTrue(m_shooter.commonShootCommand(4.0, true));

    m_driverJoystick
        .rightTrigger()
        .whileTrue(Commands.waitSeconds(0.5).andThen(m_intake.outPut(-60)))
        .whileTrue(m_shooter.differentialShootDownCommand());

    // m_driverJoystick
    // .x()
    // .whileTrue(m_intake.smartUpTake(30))
    // .onFalse(
    // Commands.waitSeconds(2.0)
    // .andThen(Commands.runOnce(() -> m_LedStrips.setRGB(255, 0, 0))));
    m_driverJoystick
        .x()
        .onTrue(m_intake.upTake(30).alongWith(m_LedStrips.setRGB_CMD(0, 15, 254)))
        .onFalse(
            m_intake
                .outPut(4)
                .withTimeout(.325)
                .andThen(m_intake.upTake(0))
                .alongWith(m_LedStrips.setRGB_CMD(254, 0, 0)));

    m_seesNote
        .onTrue(m_LedStrips.setRGB_CMD(254, 254, 254).repeatedly())
        .onFalse(m_LedStrips.setRGB_CMD(255, 0, 0));

    // .onFalse(
    // Commands.waitSeconds(1);
    // m_LedStrips.setRGB(255, 0, 0);
    // );
    m_driverJoystick
        .a()
        .whileTrue(m_intake.outPut())
        .whileTrue(m_shooter.commonShootCommand(20.0, true));

    m_driverJoystick
        .b()
        .whileTrue(m_intake.outPut(-40.0))
        .whileTrue(m_shooter.commonShootCommand(50.0, false))
        .onFalse(m_arm.armBackZero());

    // m_joystick.b().whileTrue(m_intake.upTake(15.0)).whileTrue(m_shooter.commonShootCommand());

    // m_driverJoystick.y().whileTrue(new VelocityShootCommand(m_intake,
    // m_shooter));
    m_driverJoystick.y().whileTrue(m_intake.outPut(-40));

    //////////////////////////// Climber ////////////////////////////
    // m_driverJoystick.povRight().whileTrue(m_rightClimber.Down());
    // m_driverJoystick.povLeft().whileTrue(m_leftClimber.Down());
    // m_driverJoystick.povUp().whileTrue(new ClimberAutoCommand(m_leftClimber,
    //////////////////////////// m_rightClimber,
    // "UP"));
    // m_driverJoystick.povDown().whileTrue(new ClimberAutoCommand(m_leftClimber,
    // m_rightClimber,
    // "DOWN"));
    m_driverJoystick.povLeft().whileTrue(new PositionClimbLeftPID(climberleft, -300));
    m_driverJoystick.povRight().whileTrue(new PositionClimbRightPID(climberright, -300));
    m_driverJoystick
        .povUp()
        .whileTrue(
            new PositionClimbLeftPID(climberleft, 400)
                .alongWith(new PositionClimbRightPID(climberright, 400)));
    m_driverJoystick
        .povDown()
        .whileTrue(
            new PositionClimbLeftPID(climberleft, -300)
                .alongWith(new PositionClimbRightPID(climberright, -300)));

    m_operatorJoystick.povUp().onTrue(m_arm.armUp());
    m_operatorJoystick.povDown().onTrue(m_arm.armDown());
    m_operatorJoystick.povRight().onTrue(m_arm.armUpMicro());
    m_operatorJoystick.povLeft().onTrue(m_arm.armDownMicro());

    m_operatorJoystick.leftBumper().whileTrue(m_shooter.differentialShootUpCommand());
    m_operatorJoystick.leftTrigger().whileTrue(m_shooter.farShootCommand());

    // m_operatorJoystick.rightBumper().whileTrue(m_shooter.commonShootCommand());
    // m_operatorJoystick.leftTrigger().whileTrue(m_shooter.differentialShootDownCommand());
    // m_operatorJoystick.rightTrigger().whileTrue(m_shooter.farShootCommand());

    m_operatorJoystick
        .x()
        .whileTrue(m_arm.armPod())
        .whileTrue(m_shooter.differentialShootDownCommand())
        .onFalse(m_arm.armBackZero());

    m_operatorJoystick.y().onTrue(m_arm.armAMP());
    // operatorController.a().onTrue(m_arm.armMid());
    // operatorController.b().onTrue(m_arm.armPod());
    m_operatorJoystick
        .a()
        .whileTrue(m_arm.armMid())
        .whileTrue(m_shooter.farShootCommand())
        .onFalse(m_arm.armBackZero());
    ;
    m_operatorJoystick
        .b()
        .whileTrue(m_arm.armPod())
        .whileTrue(m_shooter.farShootCommand())
        .onFalse(m_arm.armBackZero());
    ;
    m_operatorJoystick.rightBumper().onTrue(m_arm.armBackZero());

    // new Trigger(() -> m_operatorJoystick.getLeftY() < -0.2 ? true : false)
    // .whileTrue new PositionClimbLeftPID(climberleft, -300);
    // new Trigger(() -> m_operatorJoystick.getLeftY() > 0.2 ? true : false)
    // .whileTrue(new PositionClimbLeftPID(climberleft, 300));
    // new Trigger(() -> m_operatorJoystick.getRightY() < -0.2 ? true : false)
    // .whileTrue(m_rightClimber.Up());
    // new Trigger(() -> m_operatorJoystick.getRightY() > 0.2 ? true : false)
    // .whileTrue(m_rightClimber.Down());
  }

  private void onFalse(Command armBackZero) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'onFalse'");
  }

  @Override
  public void initSendable(SendableBuilder builder) {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return drivetrain
        .runOnce(() -> drivetrain.seedFieldRelative(new Pose2d()))
        .andThen(autoChooser.getSelected());
  }
}
