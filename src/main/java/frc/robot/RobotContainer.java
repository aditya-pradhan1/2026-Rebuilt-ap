// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
// import frc.robot.generated.TunerConstants;
import frc.robot.Constants.*;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.*;

public class RobotContainer {
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDeadband(ControllerConstants.kDeadband).withRotationalDeadband(ControllerConstants.kDeadband) // Add a
                                                                                                               // 10%
                                                                                                               // deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(DrivetrainConstants.MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(ControllerConstants.kDriverPort);
    private final CommandXboxController operator = new CommandXboxController(ControllerConstants.kOperatorPort);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final VisionSubsystem vision = new VisionSubsystem(drivetrain);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> getDriverDrivetrainInput()));

        operator.a().whileTrue(new ShooterCommand(shooter));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();

        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.b().whileTrue(drivetrain
                .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        if (Robot.isSimulation()) {
                driver.y().whileTrue(new RunCommand(() -> drivetrain.driveToPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))));
        }

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    // Generates the command request for moving the drive train based on the current
    // controller input.
    public RobotCentric getDriverDrivetrainInput() {
        double rightTriggerDepth = driver.getRightTriggerAxis();
        double leftTriggerDepth = driver.getLeftTriggerAxis();

        double netForwardAcceleration = (rightTriggerDepth - leftTriggerDepth) * DrivetrainConstants.MaxSpeed;

        double angularAcceleration = -driver.getLeftX() * DrivetrainConstants.MaxAngularRate;

        return drive
                .withVelocityX(netForwardAcceleration)
                .withRotationalRate(angularAcceleration);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
                // Reset our field centric heading to match the robot
                // facing away from our alliance station wall (0 deg).
                drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                // Then slowly drive forward (away from us) for 5 seconds.
                drivetrain.applyRequest(() -> drive.withVelocityX(0.5)
                        .withVelocityY(0)
                        .withRotationalRate(0))
                        .withTimeout(5.0),
                // Finally idle for the rest of auton
                drivetrain.applyRequest(() -> idle));
    }
}
