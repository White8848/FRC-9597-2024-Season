package frc.robot.subsystems;

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

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.Constants.DriveTrainConstants;
import frc.robot.LimelightHelpers;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private PIDController autoHeadingController = new PIDController(6.0, 0.0, 0.0);
    private double Headingtarget = 0;

    private SwerveRequest.ApplyChassisSpeeds driveAuto = new SwerveRequest.ApplyChassisSpeeds();

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(DriveTrainConstants.MaxSpeed * DriveTrainConstants.DeadBand)
            .withRotationalDeadband(DriveTrainConstants.MaxAngularRate * DriveTrainConstants.DeadBand)
            // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentricFacingAngle driveHeading = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(DriveTrainConstants.MaxSpeed * DriveTrainConstants.DeadBand)
            .withRotationalDeadband(DriveTrainConstants.MaxAngularRate * DriveTrainConstants.DeadBand)
            // Add a 5% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);

        driveHeading.HeadingController.setPID(4.0, 0.0, 0.0);

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

    public Command applyRequest(CommandXboxController Joystick, CommandXboxController operatorJoystick) {
        return run(() -> {
            if (operatorJoystick.y().getAsBoolean() == false) {
                if (Math.abs(Joystick.getLeftY()) < DriveTrainConstants.DeadBand
                        && Math.abs(Joystick.getLeftX()) < DriveTrainConstants.DeadBand
                        && Math.abs(Joystick.getRightX()) < DriveTrainConstants.RotationalDeadband) {
                    this.setControl(brake);
                } else {
                    this.setControl(drive
                            .withVelocityX(this.curveControl(-Joystick.getLeftY()) *
                                    DriveTrainConstants.MaxSpeed)
                            .withVelocityY(this.curveControl(-Joystick.getLeftX()) *
                                    DriveTrainConstants.MaxSpeed)
                            .withRotationalRate(headingControl(-Joystick.getRightX(),
                                    this.getState().Pose.getRotation().getRadians())));
                }
            } else {
                this.setControl(driveHeading
                        .withVelocityX(this.curveControl(-Joystick.getLeftY()) *
                                DriveTrainConstants.MaxSpeed)
                        .withVelocityY(this.curveControl(-Joystick.getLeftX()) *
                                DriveTrainConstants.MaxSpeed)
                        .withTargetDirection(Rotation2d.fromDegrees(90)));
                Headingtarget = this.getState().Pose.getRotation().getRadians();
            }

            SmartDashboard.putNumber("HeadingTarget", DriveTrainConstants.HeadingTarget.getRadians());
            SmartDashboard.putNumber("RealHead", this.getState().Pose.getRotation().getRadians());
        });
    }

    public Command resetFieldRelative() {
        return runOnce(() -> {
            DriveTrainConstants.HeadingTarget = this.getState().Pose.getRotation();
            this.seedFieldRelative();
        });
    }

    private double headingControl(double joystickInput, double current) {
        if (Math.abs(joystickInput) <= DriveTrainConstants.RotationalDeadband) {
            return 0;
        } else {
            joystickInput = curveControl(joystickInput);
        }
        // speed is in radians per second
        Headingtarget += joystickInput * DriveTrainConstants.MaxAngularRate * DriveTrainConstants.timestampPeriod;
        double error = Headingtarget - current;
        while (Math.abs(error) > Math.PI) {
            if (error > Math.PI) {
                error -= 2 * Math.PI;
            } else if (error < -Math.PI) {
                error += 2 * Math.PI;
            }
        }

        double output = autoHeadingController.calculate(0, error);

        return output;
    }

    @Override
    public void periodic() {
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        if (limelightMeasurement.tagCount >= 2) {
            this.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
            this.addVisionMeasurement(
                    limelightMeasurement.pose,
                    limelightMeasurement.timestampSeconds);
        }
        SmartDashboard.putNumber("VisionTagNumber", limelightMeasurement.tagCount);
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
