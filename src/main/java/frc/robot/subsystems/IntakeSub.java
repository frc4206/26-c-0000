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
  ConfigTalonFX.Config intakePivotMotorConfig = new ConfigTalonFX.Config("IntakePivotMotor.toml");

  public Config intakeConfig; 

  /* Motors */
  public TalonFX intakeRollersMotor1 = new TalonFX(intakeRollersMotor1Config.canID, "rio"); 
  public TalonFX intakeRollersMotor2 = new TalonFX(intakeRollersMotor2Config.canID, "rio"); 
  public TalonFX intakePivotMotor = new TalonFX(intakePivotMotorConfig.canID, "rio");

  ConfigTalonFX intakePivotMotor1CFGapply = new ConfigTalonFX(intakePivotMotorConfig, intakePivotMotor);

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
    intakeCANcoder = new CANcoder(intakeConfig.intakeCANcoderID, "rio"); 

    intakePivotMotor1CFGapply.setSlot0(intakePivotMotorConfig.slot0);
    intakePivotMotor1CFGapply.applyConfigs();
  }

  public void setPercentageRollers_func(double percentage) {
    intakeRollersMotor1.setControl(new DutyCycleOut(percentage)); 
    intakeRollersMotor2.setControl(new DutyCycleOut(-percentage)); 
  }

  public void setPercentagePivot_func(double percentage) {
    intakePivotMotor.setControl(new DutyCycleOut(percentage)); 
  }

  public void setPosition_func(double pos) {
    intakePivotMotor.setControl(new PositionVoltage(0).withPosition(pos).withSlot(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
