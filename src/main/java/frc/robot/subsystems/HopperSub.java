// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.team4206.battleaid.common.LoadableConfig;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.ConfigTalonFX;
import frc.robot.common.ConfigTalonFX.Config;

public class HopperSub extends SubsystemBase {
  /** Creates a new HopperSub. */
  /* Configs */
  ConfigTalonFX.Config hopperMotor1Config = new ConfigTalonFX.Config("hopperMotor1.toml"); 
  ConfigTalonFX.Config hopperMotor2Config = new ConfigTalonFX.Config("hopperMotor2.toml");

  public Config hopperConfig; 

  /* Motors */
  public TalonFX hopperMotor1 = new TalonFX(hopperMotor1Config.canID, "Default Name"); 
  public TalonFX hopperMotor2 = new TalonFX(hopperMotor2Config.canID, "Default Name"); 

  public static class Config extends LoadableConfig {
    /* Misc. */
    public double intakePercent; 
    public double outtakePercent; 

    public Config(String filename) {
      super.load(this, filename); 
      // LoadableConfig.print(this); 
    }
  }

  public HopperSub(Config hopperConfig) {
    this.hopperConfig = hopperConfig; 
  }

  public void setPercentage_func(double percentage) {
    hopperMotor1.setControl(new DutyCycleOut(percentage));
    hopperMotor2.setControl(new DutyCycleOut(percentage)); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
