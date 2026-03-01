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
public class AutoRotate_Com extends Command {
  /** Creates a new AutoRotate_Com. */
  public boolean isFinished = false; 
  PhotonCamera camera = new PhotonCamera("robovikes4206"); //PLACEHOLDER- go find real camera name
  PhotonTrackedTarget target; 
  double cameraRobotOffset = 0.0; //go back and measure 
  
  public static class Config extends LoadableConfig {

    double kpy; 
    double kiy;
    double kdy; 
    double kddiff; 

    public Config(String filename) {
      super.load(this, filename);
    }
  }
  
  double m_setpointY; 
  
  static Config cfg = new Config("Swerve.toml");
  CommandSwerveDrivetrain m_drive; 
  double MaxSpeed; 
  double MaxAngularRate; 
  double error; 
  double lastError = 0; 
  double deltaY; 
  TunedJoystick tj; 
  double targetYaw = 0.0; 

  public AutoRotate_Com(CommandSwerveDrivetrain drive, double setpointY, double speed, double angularRate, TunedJoystick _tj) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive; 
    m_setpointY = setpointY; 
    tj = _tj; 

    MaxSpeed = speed; 
    MaxAngularRate = angularRate; 
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double sag_output = 0.0;
    double xOutput = 0.0; 

    var result = camera.getLatestResult(); 
    boolean hasTargets = result.hasTargets(); 
    PhotonTrackedTarget target = result.getBestTarget(); 

    if (target != null) {
      //X is forward, Y is left, Z is up
      double distanceToAprilTag = target.getBestCameraToTarget().getX(); 
      double central_alignment = target.getBestCameraToTarget().getY() - cameraRobotOffset; 
      double targetYaw = target.getYaw(); 
      
      if (distanceToAprilTag >= -0.5d){
        sag_output = 0.0d; 
      } else {
        sag_output = -distanceToAprilTag; 
      }

      if (hasTargets) {
        central_alignment -= m_setpointY; 
        xOutput += (central_alignment*cfg.kpy); 
        double diff = central_alignment - lastError; 
        xOutput += (diff*cfg.kddiff); 
        

      }

      SwerveRequest driverequest = new SwerveRequest.RobotCentric().withSteerRequestType(SteerRequestType.Position);

    }
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
