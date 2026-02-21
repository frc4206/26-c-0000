// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.util.ClassUtil;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.IncrementSpeedTesting_Com;
import frc.robot.commands.IncrementSpeedUp_Com;
import frc.robot.commands.IntakeJoystick_Com;
import frc.robot.commands.ClimberJoystick_Com;
import frc.robot.commands.PercentCommands.*;

import frc.robot.generated.TunerConstants;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.ClimberSub;
import frc.robot.subsystems.HopperSub;
import frc.robot.subsystems.IntakeSub;

public class RobotContainer {
    /* Subsystems */
    public final ShooterSub.Config m_shooterConfig = new ShooterSub.Config("Shooter.toml");
    public final ClimberSub.Config m_climberConfig = new ClimberSub.Config("Climber.toml"); 
    public final HopperSub.Config m_hopperConfig = new HopperSub.Config("Hopper.toml"); 
    public final IntakeSub.Config m_intakeConfig = new IntakeSub.Config("Intake.toml"); 


    final ShooterSub m_shooter = new ShooterSub(m_shooterConfig); 
    final ClimberSub m_climber = new ClimberSub(m_climberConfig); 
    final HopperSub m_hopper = new HopperSub(m_hopperConfig); 
    final IntakeSub m_intake = new IntakeSub(m_intakeConfig); 


    private final CommandXboxController m_testingController = new CommandXboxController(2);
    private final CommandXboxController m_climberController = new CommandXboxController(3); 

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        drivetrain.registerTelemetry(logger::telemeterize);

        m_testingController.y().onTrue(new ShooterPercent_Com(m_shooter, 0));
        m_testingController.a().onTrue(new IncrementSpeedTesting_Com(m_shooter)); 
        m_testingController.x().onTrue(new IncrementSpeedUp_Com(m_shooter, 0.1)); 
        m_testingController.b().onTrue(new IncrementSpeedUp_Com(m_shooter, -0.1)); 

        m_testingController.rightBumper().onTrue(new HopperPercent_Com(m_hopper, 1));
        m_testingController.leftBumper().onTrue(new HopperPercent_Com(m_hopper, 0.0));

        // m_intake.setDefaultCommand(new IntakeJoystick_Com(m_intake, m_testingController));
        

        m_climber.setDefaultCommand(new ClimberJoystick_Com(m_climber, m_climberController));


    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
