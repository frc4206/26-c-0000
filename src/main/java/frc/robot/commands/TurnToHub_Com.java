// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnToHub_Com extends Command {
  /** Creates a new TurnToHub_Com. */
  CommandSwerveDrivetrain m_drive;
  VisionSub m_vision;

  PIDController turnPID = new PIDController(6.0, 0.0, 0.11); //these values are pretty good 

  public TurnToHub_Com(CommandSwerveDrivetrain drive, VisionSub vision) {
    this.m_drive = drive;
    this.m_vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive, m_vision);

    turnPID.setTolerance(2.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if there is no tag visible do not move bot & quit cmd
     if (!m_vision.hasTarget()) {
            m_drive.driveRobotRelative(new ChassisSpeeds());
            return;
        }

        double yToCent = m_vision.getHubY();
        double rotVelo = turnPID.calculate(yToCent, 0);

        //set the speeds of the robot to the calc'd rotational velo
        m_drive.driveRobotRelative(new ChassisSpeeds(0,0,-rotVelo));
      }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
