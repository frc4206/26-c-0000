// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.team4206.battleaid.common.LoadableConfig;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.ConfigTalonFX;
import frc.robot.common.ConfigTalonFX.Config;

public class ShooterSub extends SubsystemBase {
  /** Creates a new ShooterSub. */
  public double targetSpeed = 0.05; 

  /* Configs */
  ConfigTalonFX.Config shooterMotor1Config = new ConfigTalonFX.Config("ShooterMotor1.toml");
  ConfigTalonFX.Config shooterMotor2Config = new ConfigTalonFX.Config("ShooterMotor2.toml");

  public Config shooterConfig; 

  /* Motors */
  public TalonFX shooterMotor1 = new TalonFX(shooterMotor1Config.canID, "rio");
  public TalonFX shooterMotor2 = new TalonFX(shooterMotor2Config.canID, "rio"); 

  ConfigTalonFX shooterMotor1Apply = new ConfigTalonFX(shooterMotor1Config, shooterMotor1);
  ConfigTalonFX shooterMotor2Apply = new ConfigTalonFX(shooterMotor2Config, shooterMotor2);

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

    shooterMotor1Apply.applyConfigs(); 
    shooterMotor2Apply.applyConfigs(); 
    shooterMotor1Apply.setSlot0(shooterMotor1Config.slot0); 
    shooterMotor2Apply.setSlot0(shooterMotor2Config.slot0);
  }

  public void setPercentage_func(double percentage) {
    shooterMotor1.set(percentage); 
    shooterMotor2.set(-percentage);
  }

  public void incrementSpeedTesting() {
    shooterMotor1.set(targetSpeed); 
    shooterMotor2.set(-targetSpeed); 
  }

  public void incrementSpeedUp(double increment) {
    targetSpeed += increment; 
    System.out.println(targetSpeed);
  }

  public void setToVelocity(double rpm) {
    double rps = rpm / 60.0; 
    shooterMotor1.setControl(new VelocityVoltage(rps));
    shooterMotor2.setControl(new VelocityVoltage(rps));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
