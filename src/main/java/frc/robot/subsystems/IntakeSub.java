// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.team4206.battleaid.common.LoadableConfig;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.ConfigTalonFX;
import frc.robot.common.ConfigTalonFX.Config;

public class IntakeSub extends SubsystemBase {
  /** Creates a new IntakeSub. */
  /* Configs */
  ConfigTalonFX.Config intakeRollersMotor1Config = new ConfigTalonFX.Config("IntakeRollersMotor1.toml");
  ConfigTalonFX.Config intakeRollersMotor2Config = new ConfigTalonFX.Config("IntakeRollersMotor2.toml");
  ConfigTalonFX.Config intakePivotMotor1Config = new ConfigTalonFX.Config("IntakePivotMotor1.toml");
  ConfigTalonFX.Config intakePivotMotor2Config = new ConfigTalonFX.Config("IntakePivotMotor2.toml"); 

  public Config intakeConfig; 

  /* Motors */
  public TalonFX intakeRollersMotor1 = new TalonFX(intakeRollersMotor1Config.canID, "Default Name"); 
  public TalonFX intakeRollersMotor2 = new TalonFX(intakeRollersMotor2Config.canID, "Default Name"); 
  public TalonFX intakePivotMotor1 = new TalonFX(intakePivotMotor1Config.canID, "Default Name");
  public TalonFX intakePivotMotor2 = new TalonFX(intakePivotMotor2Config.canID, "Default Name"); 

  ConfigTalonFX intakePivotMotor1CFGapply = new ConfigTalonFX(intakePivotMotor1Config, intakePivotMotor1);
  ConfigTalonFX intakePivotMotor2CFGapply = new ConfigTalonFX(intakePivotMotor2Config, intakePivotMotor1);

  /* Sensors */
  CANcoder intakeCANcoder;  

  public static class Config extends LoadableConfig {

    /* IDs and Ports */
    public int intakeCANcoderID; 

    /* Positions */
    public double stowPosition; 
    public double homePosition; 
    public double intakePosition; 

    /* Misc. */
    public double intakePercent; 
    public double outtakePercent; 

    public Config(String filename) {
      super.load(this, filename); 
      //LoadableConfig.print(this); 
    }
  }

  public IntakeSub(Config intakeConfig) {
    this.intakeConfig = intakeConfig; 
    intakeCANcoder = new CANcoder(intakeConfig.intakeCANcoderID, "Default Name"); 

    intakePivotMotor1CFGapply.setSlot0(intakePivotMotor1Config.slot0);
    intakePivotMotor2CFGapply.setSlot0(intakePivotMotor2Config.slot0); 
    intakePivotMotor1CFGapply.applyConfigs();
    intakePivotMotor2CFGapply.applyConfigs();
  }

  public void setPercentage_func(double percentage) {
    intakeRollersMotor1.setControl(new DutyCycleOut(percentage)); 
    intakeRollersMotor2.setControl(new DutyCycleOut(percentage)); 
  }

  public void setPosition_func(double pos) {
    // intakePivtorMotor2.setPosition(intakePivotMotor1.getPosition().getValueAsDouble()); 
    intakePivotMotor1.setControl(new PositionVoltage(0).withPosition(pos).withSlot(0));
    intakePivotMotor2.setControl(new PositionVoltage(0).withPosition(pos).withSlot(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
