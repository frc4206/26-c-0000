// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.ejml.dense.block.decomposition.hessenberg.TridiagonalDecompositionHouseholder_DDRB;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.team4206.battleaid.common.LoadableConfig;
import org.team4206.battleaid.common.TunedJoystick;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Rotate_Com extends Command {
  /** Creates a new AutoRotate_Com. */
  public boolean isFinished = false; 
  PhotonCamera camera = new PhotonCamera("frontcam"); 
  PhotonTrackedTarget target; 
  double cameraRobotOffset = 0.0; //go back and measure 
  
  public static class Config extends LoadableConfig {


    public Config(String filename) {
      super.load(this, filename);
    }
  }
  
  double m_setpointY; 
  
  static Config cfg = new Config("Swerve.toml");
  CommandSwerveDrivetrain m_drive; 


  public Rotate_Com(CommandSwerveDrivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    drive = m_drive; 

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // var result = camera.getLatestResult(); 
    // boolean hasTargets = result.hasTargets(); 
    // PhotonTrackedTarget target = result.getBestTarget(); 

    // if (target != null) {
    //   double yaw_of_qr_code = target.getYaw(); 

    // }

    // SwerveRequest.RobotCentric driverequest = new SwerveRequest.RobotCentric().
    //       withSteerRequestType(null).withRotationalRate(0.5);

    // // if (!hasTargets && target.getYaw() < 0) {

    // // }

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
