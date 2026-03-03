// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.team4206.battleaid.common.LoadableConfig;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.compound.Diff_VelocityDutyCycle_Velocity;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.ConfigTalonFX;
import frc.robot.common.ConfigTalonFX.Config;

public class ShooterSub extends SubsystemBase {
  /** Creates a new ShooterSub. */
  public double targetSpeed = 0.5;

  public double VOLTAGE_TO_OVERCOME_STATIC_FRICTION = 0.22d; // VOLTS, tested by hand, DO NOT change
  public double VOLTAGE_TO_MAINTAIN_SPEED = 0.15d;

  /* Configs */
  ConfigTalonFX.Config shooterMotor1Config = new ConfigTalonFX.Config("ShooterMotor1.toml");
  ConfigTalonFX.Config shooterMotor2Config = new ConfigTalonFX.Config("ShooterMotor2.toml");

  public Config shooterConfig;

  /* Motors */
  public TalonFX shooterMotor1 = new TalonFX(shooterMotor1Config.canID, "rio");
  public TalonFX shooterMotor2 = new TalonFX(shooterMotor2Config.canID, "rio");

  ConfigTalonFX shooterMotor1Apply = new ConfigTalonFX(shooterMotor1Config, shooterMotor1);
  ConfigTalonFX shooterMotor2Apply = new ConfigTalonFX(shooterMotor2Config, shooterMotor2);

  TalonFXConfiguration motor1config = new TalonFXConfiguration();
  TalonFXConfiguration motor2config = new TalonFXConfiguration();

  public static class Config extends LoadableConfig {

    /* Misc */
    public double outtakePercent; // placeholder until treetable stuff happens

    public Config(String filename) {
      super.load(this, filename);
      // LoadableConfig.print(this);
    }
  }

  public ShooterSub(Config shooterConfig) {
    this.shooterConfig = shooterConfig;

    // shooterMotor1Apply.applyConfigs();

    motor1config.Slot0.kS = VOLTAGE_TO_OVERCOME_STATIC_FRICTION;
    motor1config.Slot0.kV = VOLTAGE_TO_MAINTAIN_SPEED;
    motor1config.Slot0.kP = 4.0;
    motor1config.Slot0.kI = 0;
    motor1config.Slot0.kD = 0.0;
    motor1config.CurrentLimits.SupplyCurrentLimit = 60;
    motor1config.CurrentLimits.SupplyCurrentLimitEnable = true;
    motor1config.CurrentLimits.StatorCurrentLimit = 90;
    motor1config.CurrentLimits.StatorCurrentLimitEnable = true;

    motor1config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    shooterMotor1.getConfigurator().apply(motor1config);

    motor2config.Slot0.kS = VOLTAGE_TO_OVERCOME_STATIC_FRICTION;
    motor2config.Slot0.kV = VOLTAGE_TO_MAINTAIN_SPEED;
    motor2config.Slot0.kP = 4.0;
    motor2config.Slot0.kI = 0;
    motor2config.Slot0.kD = 0.0;
    motor2config.CurrentLimits.SupplyCurrentLimit = 60;
    motor2config.CurrentLimits.SupplyCurrentLimitEnable = true;
    motor2config.CurrentLimits.StatorCurrentLimit = 90;
    motor2config.CurrentLimits.StatorCurrentLimitEnable = true;

    motor2config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shooterMotor2.getConfigurator().apply(motor2config);

    // shooterMotor1Apply

    

    shooterMotor1Apply.setSlot0(shooterMotor1Config.slot0); 
    shooterMotor2Apply.setSlot0(shooterMotor2Config.slot0);
    shooterMotor1Apply.applyConfigs(); 
    shooterMotor2Apply.applyConfigs(); 
  }

  public void setPercentage_func(double percentage) {
    shooterMotor1.set(percentage);
    shooterMotor2.set(-percentage);
  }

  public void incrementSpeedTesting() {
    // shooterMotor1.set(targetSpeed);
    // shooterMotor2.set(-targetSpeed);
    shooterMotor1.setControl(new DutyCycleOut(targetSpeed));
    shooterMotor2.setControl(new DutyCycleOut(-targetSpeed));
  }

  public void incrementSpeedUp(double increment) {
    targetSpeed += increment;
    System.out.println("VELOCITY: " + targetSpeed);
  }

  public void setFlywheelSpeed(double velocity) {
    // the first VelocityVoltage value is 0 because the withVelocity value replaces
    // it anyway,
    // and the withVelocity value is necessary for feedforward. If you take
    // feedforward out
    // then just delete until withSlot and replace the '0' with 'velocity'
    velocity = velocity / 60; // to get to rps from rpm

    shooterMotor1.setControl(new VelocityVoltage(velocity).withSlot(0));
    shooterMotor2.setControl(new VelocityVoltage(velocity).withSlot(0));
  }

  public void setFlywheelVoltage(double voltage) {
    VoltageOut voltageRequest = new VoltageOut(0.1);
    shooterMotor1.setControl(voltageRequest.withOutput(voltage));
    shooterMotor2.setControl(voltageRequest.withOutput(voltage));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println("VELOCITY: " + targetSpeed);
  }

  /** Set the target velocity of the shooter in motor units per 100ms */
  public void setFlywheelVelocity(double targetRPM) {
    // Convert RPM to TalonFX native units (ticks per 100ms)
    double ticksPer100ms = rpmToTicksPer100ms(targetRPM);

    // Use TalonFX VelocityVoltage control
    VelocityVoltage control = new VelocityVoltage(ticksPer100ms);
    shooterMotor1.setControl(control);
    shooterMotor2.setControl(control);
  }

  /** Stop the shooter motors */
  public void stop() {
    shooterMotor1.setControl(new VoltageOut(0));
    shooterMotor2.setControl(new VoltageOut(0));
  }

  /** Helper: Convert RPM to TalonFX units per 100ms */
  private double rpmToTicksPer100ms(double rpm) {
    double ticksPerRev = 2048.0; // Falcon 500 encoder ticks per revolution
    return rpm * ticksPerRev / 600.0; // 600 = 60s * 10 (100ms per tick)
  }

  /** Optional: get current RPM for debugging / PID monitoring */
  // public double getCurrentRPM() {
  // double sensorVel1 = shooterMotor1.getRotorVelocity().getValue();
  // return sensorVelToRPM(sensorVel1);
  // }

  // private double sensorVelToRPM(double ticksPer100ms) {
  // return ticksPer100ms * 600.0 / 2048.0;
  // }
}
