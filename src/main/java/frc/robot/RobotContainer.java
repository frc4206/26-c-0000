// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.util.ClassUtil;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.IncrementSpeedTesting_Com;
import frc.robot.commands.IncrementSpeedUp_Com;
import frc.robot.commands.IntakeJoystick_Com;
import frc.robot.commands.RunFlywheelVoltage;
import frc.robot.commands.SetFlywheelSpeed_Com;
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
    // public final ShooterSub.Config m_shooterConfig = new
    // ShooterSub.Config("Shooter.toml");
    // public final ClimberSub.Config m_climberConfig = new
    // ClimberSub.Config("Climber.toml");
    // public final HopperSub.Config m_hopperConfig = new
    // HopperSub.Config("Hopper.toml");
    // public final IntakeSub.Config m_intakeConfig = new
    // IntakeSub.Config("Intake.toml");

    // final ShooterSub m_shooter = new ShooterSub(m_shooterConfig);
    // final ClimberSub m_climber = new ClimberSub(m_climberConfig);
    // final HopperSub m_hopper = new HopperSub(m_hopperConfig);
    // final IntakeSub m_intake = new IntakeSub(m_intakeConfig);

    /* Joysticks */
    // private final CommandXboxController m_driverController = new
    // CommandXboxController(0);
    // private final CommandXboxController m_testingController = new
    // CommandXboxController(2);
    // private final CommandXboxController m_climberController = new
    // CommandXboxController(3);
    public PowerDistribution pdh = new PowerDistribution();
    public VoltageMonitor voltageMonitor = new VoltageMonitor(0.017d);

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

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
        /* Pathplanner Named Commands */

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // drivetrain.setDefaultCommand(
        // // Drivetrain will execute this command periodically
        // drivetrain.applyRequest(() ->
        // drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
        // negative Y (forward)
        // .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X
        // (left)
        // .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive
        // counterclockwise with negative X (left)
        // )
        // );

        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> {

                    double resistance = 0.017d;
                    double voltageFloor = 10.0d;

                    double y = joystick.getLeftY();
                    double x = joystick.getLeftX();

                    double inputMag = Math.sqrt(x * x + y * y);
                    double angle = Math.atan2(y, x);

                    double num_motors = 4;
                    double motorCurrentLimit = 50;

                    double currentVoltage = RobotController.getBatteryVoltage();
                    double currentAmps = pdh.getTotalCurrent();

                    // double rest_voltage = currentVoltage + (currentAmps * resistance);


                    System.out.printf(
                            "Measured Voltage %.3f, Measured Amps %.3f, Estimated Voltage: %.3f\n",
                            currentVoltage,
                            currentAmps,
                            voltageMonitor.getUnloadedVoltage());

                    // double expectedVoltageDrop = (num_motors * motorCurrentLimit) * resistance;
                    // double expectedVoltage = currentVoltage - expectedVoltageDrop;

                    // double outputMagnitude = inputMag;
                    // if (expectedVoltage < voltageFloor) {
                    // double numerator = currentVoltage - voltageFloor;
                    // double denominator = currentVoltage - expectedVoltage;
                    // double magnitudeMultiplier = numerator / denominator;

                    // System.out.printf("Voltage adjust: %.3f / %.3f = %.3f%n",
                    // numerator, denominator, magnitudeMultiplier);

                    // outputMagnitude = inputMag * magnitudeMultiplier;
                    // }

                    // System.out.printf("Magnitudes: input = %.3f, output = %.3f%n",
                    // inputMag, outputMagnitude);

                    // // convert magnitude back into x and y and also remember plus or minus
                    // double x_out = outputMagnitude * Math.cos(angle);
                    // double y_out = outputMagnitude * Math.sin(angle);

                    // System.out.printf("Output vector: x = %.3f, y = %.3f%n\n\n",
                    // x_out, y_out);

                    double y_out = y;
                    double x_out = x;

                    return drive.withVelocityX(-y_out * MaxSpeed)
                            .withVelocityY(-x_out * MaxSpeed)
                            .withRotationalRate(-joystick.getRightX() * MaxAngularRate);
                }));

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
