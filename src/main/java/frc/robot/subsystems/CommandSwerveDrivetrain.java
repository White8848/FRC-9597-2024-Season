package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.DraveTrainConstants;;
/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private SwerveRequest.ApplyChassisSpeeds driveAuto = new SwerveRequest.ApplyChassisSpeeds();

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(DraveTrainConstants.MaxSpeed * DraveTrainConstants.DeadBand).withRotationalDeadband(DraveTrainConstants.MaxAngularRate * DraveTrainConstants.DeadBand) // Add a 5% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentricFacingAngle driveHeading = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(DraveTrainConstants.MaxSpeed * DraveTrainConstants.DeadBand).withRotationalDeadband(DraveTrainConstants.MaxAngularRate * DraveTrainConstants.DeadBand) // Add a 5% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);

        driveHeading.HeadingController.setPID(5.0, 0.0, 0.0);

        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Pose2d getPose() {
        return this.getState().Pose;
    }

    public void resetPose(Pose2d newPose) {
        this.seedFieldRelative(newPose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return this.m_kinematics.toChassisSpeeds(this.getState().ModuleStates);
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        this.setControl(driveAuto.withSpeeds(speeds));
    }

    public Rotation2d getRotation() {
        return this.getState().Pose.getRotation();
    }

    public Command applyRequest(CommandXboxController Joystick) {
        return run(() -> {
            if (Math.abs(Joystick.getRightX()) >= DraveTrainConstants.DeadBand) {
                this.setControl(drive
                        .withVelocityX(this.curveControl(-Joystick.getLeftY()) * DraveTrainConstants.MaxSpeed)
                        .withVelocityY(this.curveControl(-Joystick.getLeftX()) * DraveTrainConstants.MaxSpeed)
                        .withRotationalRate(this.curveControl(-Joystick.getRightX()) * DraveTrainConstants.MaxAngularRate));
                DraveTrainConstants.HeadingTarget = this.getRotation();
            } else {
                this.setControl(driveHeading
                        .withVelocityX(this.curveControl(-Joystick.getLeftY()) * DraveTrainConstants.MaxSpeed)
                        .withVelocityY(this.curveControl(-Joystick.getLeftX()) * DraveTrainConstants.MaxSpeed)
                        .withTargetDirection(DraveTrainConstants.HeadingTarget));
            }
        });
    }

    public Command resetHead() {
        return runOnce(() -> {
            DraveTrainConstants.HeadingTarget = this.getRotation();
            this.seedFieldRelative();
        });
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    private double curveControl(double input) {
        if (input < 0) {
            return -Math.pow(input, 2);
        }
        return Math.pow(input, 2);
    }
}
