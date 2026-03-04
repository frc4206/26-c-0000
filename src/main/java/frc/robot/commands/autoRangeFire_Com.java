// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.VisionSub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class autoRangeFire_Com extends Command {
  /** Creates a new autoRangeFire_Com. */
  ShooterSub m_shooter;
  VisionSub m_vision;
  public autoRangeFire_Com(ShooterSub shooter, VisionSub vision) {
    m_shooter = shooter;
    m_vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter, m_vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_vision.hasTarget()) {
    double distance = m_vision.getTargetX();
    System.out.println("Distance: " + distance);
    m_shooter.autoRangeFire_func(distance);
}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
