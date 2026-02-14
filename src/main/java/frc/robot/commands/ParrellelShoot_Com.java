// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.checkerframework.checker.units.qual.h;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSub;
import frc.robot.subsystems.ShooterSub;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ParrellelShoot_Com extends Command {
  /** Creates a new ParrellelShoot_Com. */
  HopperSub m_hopperSub; 
  ShooterSub m_shooterSub; 

  public ParrellelShoot_Com(HopperSub hopperSub, ShooterSub shooterSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_hopperSub = hopperSub; 
    m_shooterSub = shooterSub; 
    addRequirements(hopperSub);
    addRequirements(shooterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_hopperSub.setPercentage_func(m_hopperSub.hopperConfig.intakePercent);
    m_shooterSub.incrementSpeedTesting(); //eventually change but for testing we're doing this command
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
