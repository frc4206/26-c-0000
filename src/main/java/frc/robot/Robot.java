// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    static SwerveModule<TalonFX, TalonFX, CANcoder>[] modules = RobotContainer.drivetrain.getModules();

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
            .withTimestampReplay()
            .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotInit() {
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());

        Logger.addDataReceiver(new WPILOGWriter()); // write logs to /U/logs
        Logger.addDataReceiver(new NT4Publisher()); // live NT view (optional)

        Logger.start();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
        m_robotContainer.periodic();

        Logger.recordOutput("Drive/FrontLeftStatorCurrent",
                modules[0].getDriveMotor().getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Steer/FrontLeftStatorCurrent",
                modules[0].getSteerMotor().getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Drive/FrontRightStatorCurrent",
                modules[1].getDriveMotor().getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Steer/FrontRightStatorCurrent",
                modules[1].getSteerMotor().getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Drive/BackLeftStatorCurrent",
                modules[2].getDriveMotor().getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Steer/BackLeftStatorCurrent",
                modules[2].getSteerMotor().getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Drive/BackRightStatorCurrent",
                modules[3].getDriveMotor().getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Steer/BackRightStatorCurrent",
                modules[3].getSteerMotor().getStatorCurrent().getValueAsDouble());

        Logger.recordOutput("Drive/FrontLeftSupplyCurrent",
                modules[0].getDriveMotor().getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("Steer/FrontLeftSupplyCurrent",
                modules[0].getSteerMotor().getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("Drive/FrontRightSupplyCurrent",
                modules[1].getDriveMotor().getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("Steer/FrontRightSupplyCurrent",
                modules[1].getSteerMotor().getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("Drive/BackLeftSupplyCurrent",
                modules[2].getDriveMotor().getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("Steer/BackLeftSupplyCurrent",
                modules[2].getSteerMotor().getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("Drive/BackRightSupplyCurrent",
                modules[3].getDriveMotor().getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("Steer/BackRightSupplyCurrent",
                modules[3].getSteerMotor().getSupplyCurrent().getValueAsDouble());
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
