// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.team4206.battleaid.common.LoadableConfig;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.ConfigTalonFX;
import frc.robot.common.ConfigTalonFX.Config;

public class ShooterSub extends SubsystemBase {
  /** Creates a new ShooterSub. */
  /* Configs */
  ConfigTalonFX.Config shooterMotor1Config = new ConfigTalonFX.Config("ShooterMotor1.toml");
  ConfigTalonFX.Config shooterMotor2Config = new ConfigTalonFX.Config("ShooterMotor2.toml");

  public Config shooterConfig; 

  /* Motors */
  public TalonFX shooterMotor1 = new TalonFX(shooterMotor1Config.canID, "Default Name");
  public TalonFX shooterMotor2 = new TalonFX(shooterMotor2Config.canID, "Default Name"); 

  public static class Config extends LoadableConfig {
    
    /* Misc */
    public double outtakePercent; //placeholder until treetable stuff happens

    public Config(String filename) {
      super.load(this, filename); 
      //LoadableConfig.print(this); 
    }
  }

  public ShooterSub(Config shooterConfig) {
    this.shooterConfig = shooterConfig; 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
