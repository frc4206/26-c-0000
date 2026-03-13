// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.util.ClassUtil;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.IncrementSpeedUp_Com;
import frc.robot.commands.IntakeJoystick_Com;
import frc.robot.commands.IntakePID_Com;
import frc.robot.commands.SetFlywheelSpeed_Com;
import frc.robot.commands.autoRangeFire_Com;
import frc.robot.commands.ClimberJoystick_Com;
import frc.robot.commands.ClimberPID_Com;
import frc.robot.commands.IncrementSpeedTesting_Com;
import frc.robot.commands.PercentCommands.*;

import frc.robot.generated.TunerConstants;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.VisionSub;
import frc.robot.subsystems.ClimberSub;
import frc.robot.subsystems.HopperSub;
import frc.robot.subsystems.IntakeSub;

public class RobotContainer {
    /* Subsystems */
    public final ShooterSub.Config m_shooterConfig = new ShooterSub.Config("Shooter.toml");
    public final ClimberSub.Config m_climberConfig = new ClimberSub.Config("Climber.toml");
    public final HopperSub.Config m_hopperConfig = new HopperSub.Config("Hopper.toml");
    public final IntakeSub.Config m_intakeConfig = new IntakeSub.Config("Intake.toml");

    final VisionSub m_vision = new VisionSub("frontcam");

    final ShooterSub m_shooter = new ShooterSub(m_shooterConfig, m_vision); 
    final ClimberSub m_climber = new ClimberSub(m_climberConfig); 
    final HopperSub m_hopper = new HopperSub(m_hopperConfig); 
    final IntakeSub m_intake = new IntakeSub(m_intakeConfig);

    /* Joysticks */
    private final CommandXboxController m_driverController = new CommandXboxController(0);
    private final CommandXboxController m_operatorController = new CommandXboxController(1);
    private final CommandXboxController m_testingController = new CommandXboxController(2);
    private final CommandXboxController m_climberController = new CommandXboxController(3);

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

    private double m_targetRPM = 2000; //2000

    private final SendableChooser<Command> autoChooser; 

    public RobotContainer() {
        /* Pathplanner Named Commands */
        /* Basic: */
        NamedCommands.registerCommand("Hopper", new HopperPercent_Com(m_hopper, 0.80).withTimeout(15.0));
        NamedCommands.registerCommand("Flywheels", new ShooterPercent_Com(m_shooter, .38).withTimeout(15.0));

        /* Human Player Auto: */
        NamedCommands.registerCommand("FlywheelsTrench", new ShooterPercent_Com(m_shooter, .51).withTimeout(5.0));
        NamedCommands.registerCommand("HopperShort", new HopperPercent_Com(m_hopper, 0.90).withTimeout(5));



        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser); 
    }


    private void configureBindings() {

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                                   // negative Y
                                                                                                   // (forward)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                    // negative X (left)
                ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        // joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        // drivetrain.registerTelemetry(logger::telemeterize);

        /* Button Bindings */
        m_driverController.rightBumper().toggleOnTrue(new HopperPercent_Com(m_hopper, 1));
        joystick.b().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric)); 
        

        // m_climber.setDefaultCommand(new ClimberJoystick_Com(m_climber, m_operatorController)); //Left stick
        m_intake.setDefaultCommand(new IntakeJoystick_Com(m_intake, m_operatorController));    //Right stick 
        m_operatorController.rightBumper().toggleOnTrue(new IntakePercent_Com(m_intake, 1)); 
        // m_operatorController.leftBumper().onTrue(new IntakePercent_Com(m_intake, 0.0));
        m_operatorController.y().onTrue(new ShooterPercent_Com(m_shooter, 0.0)); 
        // m_operatorController.rightTrigger().onTrue(new IncrementSpeedTesting_Com(m_shooter));
        m_operatorController.leftTrigger().onTrue(new IntakePercent_Com(m_intake, -0.8));
        //This is just in case
        // m_operatorController.x().onTrue(new IncrementSpeedUp_Com(m_shooter, 0.025)); 
        // m_operatorController.b().onTrue(new IncrementSpeedUp_Com(m_shooter, -0.025));
        /* Increment target RPM */
        m_operatorController.x().onTrue(
                new InstantCommand(() -> {
                    m_targetRPM += 25;
                    System.out.println("Right Bumper Pressed → Target RPM: " + m_targetRPM);
                }));

        /* Decrement target RPM */
        m_operatorController.b().onTrue(
                new InstantCommand(() -> {
                    m_targetRPM -= 25;
                    System.out.println("Left Bumper Pressed → Target RPM: " + m_targetRPM);
                }));

        //Changed from whileTrue to onTrue 
        m_operatorController.a().onTrue(
            new SetFlywheelSpeed_Com(m_shooter, () -> m_targetRPM)
        );
        

        m_operatorController.pov(0).whileTrue(new autoRangeFire_Com(m_shooter, m_vision));


        // m_testingController.y().onTrue(new ShooterPercent_Com(m_shooter, 0)); // STOP CHANGING THIS TO 50. IF YOU WANT
                                                                              // IT TO GO TO 50 PRESS A AND INCREMENT
                                                                              // LIKE A NORMAL PERSON. THIS BUTTON IS TO
                                                                              // STOP THE SHOOTER
        m_testingController.a().onTrue(new IncrementSpeedTesting_Com(m_shooter));
        m_testingController.x().onTrue(new IncrementSpeedUp_Com(m_shooter, 0.01));
        m_testingController.b().onTrue(new IncrementSpeedUp_Com(m_shooter, -0.01));



        // m_testingController.a().onTrue(new SetFlywheelSpeed_Com(m_shooter,4000));
        // m_testingController.a().whileTrue(new RunFlywheelVoltage(m_shooter, 0.1));

        // m_testingController.rightBumper().onTrue(new HopperPercent_Com(m_hopper, 0.8));
        // m_testingController.leftBumper().onTrue(new HopperPercent_Com(m_hopper, 0.0));
        // m_testingController.rightTrigger().onTrue(new IntakePercent_Com(m_intake,
        // 0.3));
        // m_testingController.leftTrigger().onTrue(new IntakePercent_Com(m_intake,
        // 0.0));
        // m_testingController.y().onTrue(new ShooterPercent_Com(m_shooter, 50));
        // m_testingController.a().onTrue(new IncrementSpeedTesting_Com(m_shooter)); 
        // m_testingController.x().onTrue(new IncrementSpeedUp_Com(m_shooter, 0.03)); 
        // m_testingController.b().onTrue(new IncrementSpeedUp_Com(m_shooter, -0.03)); 
        // m_testingController.a().onTrue(new SetFlywheelSpeed_Com(m_shooter,4000));
        // m_testingController.a().whileTrue(new RunFlywheelVoltage(m_shooter, 0.1));
        // m_testingController.pov(0).whileTrue(new autoRangeFire_Com(m_shooter, m_vision));

        // m_testingController.rightTrigger().onTrue(new IntakePercent_Com(m_intake, 0.5));
        // m_testingController.leftTrigger().onTrue(new IntakePercent_Com(m_intake, 0.0));
        // m_intake.setDefaultCommand(new IntakeJoystick_Com(m_intake, m_testingController));

        // m_driverController.rightTrigger().onTrue(new HopperPercent_Com(m_hopper,
        // 0.9));
        // m_driverController.leftTrigger().onTrue(new HopperPercent_Com(m_hopper,
        // 0.0));

        // m_climber.setDefaultCommand(new ClimberJoystick_Com(m_climber, m_climberController));

        m_driverController.a().onTrue(new InstantCommand(() -> drivetrain.getPigeon2().reset()));
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        // final var idle = new SwerveRequest.Idle();
        // return Commands.sequence(
        //         // Reset our field centric heading to match the robot
        //         // facing away from our alliance station wall (0 deg).
        //         drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
        //         // Then slowly drive forward (away from us) for 5 seconds.
        //         drivetrain.applyRequest(() -> drive.withVelocityX(0.5)
        //                 .withVelocityY(0)
        //                 .withRotationalRate(0))
        //                 .withTimeout(5.0),
        //         // Finally idle for the rest of auton
        //         drivetrain.applyRequest(() -> idle));

        return autoChooser.getSelected(); 
    }
}
