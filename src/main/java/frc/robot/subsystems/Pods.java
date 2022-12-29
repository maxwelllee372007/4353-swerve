// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.net.CacheRequest;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pods extends SubsystemBase {

  CANCoder c1;
  CANCoder c2;
  CANCoder c3;
  CANCoder c4;

  TalonFX s1;
  TalonFX s2;
  TalonFX s3;
  TalonFX s4;

  TalonFX d1;
  TalonFX d2;
  TalonFX d3;
  TalonFX d4;

  Pigeon2 gyro; // Pigeon is on CAN Bus with device ID 0
  
  /** Creates a new Pods. */
  public Pods() {
    c1 = new CANCoder(Constants.c1);
    c2 = new CANCoder(Constants.c2);
    c3 = new CANCoder(Constants.c3);
    c4 = new CANCoder(Constants.c4);

    s1 = new TalonFX(Constants.s1);
    s2 = new TalonFX(Constants.s2);
    s3 = new TalonFX(Constants.s3);
    s4 = new TalonFX(Constants.s4);

    d1 = new TalonFX(Constants.d1);
    d2 = new TalonFX(Constants.d2);
    d3 = new TalonFX(Constants.d3);
    d4 = new TalonFX(Constants.d4);

    gyro = new Pigeon2(Constants.pigeonPort);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("s1 pos", c1get());
    SmartDashboard.putNumber("s2 pos", c2get());
    SmartDashboard.putNumber("s3 pos", c3get());
    SmartDashboard.putNumber("s4 pos", c4get());
  }

  /** returns in ticks (43008 per rev) */
  public double s1get() {
    return s1.getSelectedSensorPosition(0);
  }
  /** returns in ticks (43008 per rev) */
  public double s2get() {
    return s2.getSelectedSensorPosition(0);
  }
  /** returns in ticks (43008 per rev) */
  public double s3get() {
    return s3.getSelectedSensorPosition(0);
  }
  /** returns in ticks (43008 per rev) */
  public double s4get() {
    return s4.getSelectedSensorPosition(0);
  }

  /** set angle in ticks (43008 per rev) */
  public void s1set(double angle) {
    s1.set(TalonFXControlMode.Position, angle);
  }
  /** set angle in ticks (43008 per rev) */
  public void s2set(double angle) {
    s2.set(TalonFXControlMode.Position, angle);
  }
  /** set angle in ticks (43008 per rev) */
  public void s3set(double angle) {
    s3.set(TalonFXControlMode.Position, angle);
  }
  /** set angle in ticks (43008 per rev) */
  public void s4set(double angle) {
    s4.set(TalonFXControlMode.Position, angle);
  }
  
  /** set velocity in ticks per 100ms(0.1s) using built in closed loop */
  public void d1set(double velocity) {
    d1.set(TalonFXControlMode.Velocity, velocity);
  }
  /** set velocity in ticks per 100ms(0.1s) using built in closed loop */
  public void d2set(double velocity) {
    d2.set(TalonFXControlMode.Velocity, velocity);
  }
  /** set velocity in ticks per 100ms(0.1s) using built in closed loop */
  public void d3set(double velocity) {
    d3.set(TalonFXControlMode.Velocity, velocity);
  }
  /** set velocity in ticks per 100ms(0.1s) using built in closed loop */
  public void d4set(double velocity) {
    d4.set(TalonFXControlMode.Velocity, velocity);
  }
  
  /** set encoder angle in ticks (43008 per rev) */  
  public void s1sensorset(double sensorPos) {
    s1.setSelectedSensorPosition(sensorPos);
  }  
  /** set encoder angle in ticks (43008 per rev) */  
  public void s2sensorset(double sensorPos) {
    s2.setSelectedSensorPosition(sensorPos);
  }
  /** set encoder angle in ticks (43008 per rev) */  
  public void s3sensorset(double sensorPos) {
    s3.setSelectedSensorPosition(sensorPos);
  }
  /** set encoder angle in ticks (43008 per rev) */  
  public void s4sensorset(double sensorPos) {
    s4.setSelectedSensorPosition(sensorPos);
  }

  /** reset pods to cancoder values and gyro to 0 */
  public void reset() {
    // c1.setPosition((c1get()+Constants.c1offset)*43008/360);
    // c2.setPosition((c2get()+Constants.c2offset)*43008/360);
    // c3.setPosition((c3get()+Constants.c3offset)*43008/360);
    // c4.setPosition((c4get()+Constants.c4offset)*43008/360);
    gyro.setYaw(0);
    d1.configClosedloopRamp(Constants.rampRateDrive);
    d2.configClosedloopRamp(Constants.rampRateDrive);
    d3.configClosedloopRamp(Constants.rampRateDrive);
    d4.configClosedloopRamp(Constants.rampRateDrive);
    s1.configClosedloopRamp(Constants.rampRateSteer);
    s2.configClosedloopRamp(Constants.rampRateSteer);
    s3.configClosedloopRamp(Constants.rampRateSteer);
    s4.configClosedloopRamp(Constants.rampRateSteer);
    // s1.config_kP(Constants.s1, Constants.STEER_P);
    // s1.config_kI(Constants.s1, Constants.STEER_I);
    // s1.config_kD(Constants.s1, Constants.STEER_D);
    // s1.config_kF(Constants.s1, Constants.STEER_F);
    // s2.config_kP(Constants.s2, Constants.STEER_P);
    // s2.config_kD(Constants.s2, Constants.STEER_D);
    // s2.config_kF(Constants.s2, Constants.STEER_F);
    // s2.config_kI(Constants.s2, Constants.STEER_I);
    // s3.config_kP(Constants.s3, Constants.STEER_P);
    // s3.config_kI(Constants.s3, Constants.STEER_I);
    // s3.config_kD(Constants.s3, Constants.STEER_D);
    // s3.config_kF(Constants.s3, Constants.STEER_F);
    // s4.config_kP(Constants.s4, Constants.STEER_P);
    // s4.config_kI(Constants.s4, Constants.STEER_I);
    // s4.config_kD(Constants.s4, Constants.STEER_D);
    // s4.config_kF(Constants.s4, Constants.STEER_F);
  }


  /** returns cancoder angle in degrees [0, 360) */
  public double c1get() {
    return c1.getAbsolutePosition();
  }
  /** returns cancoder angle in degrees [0, 360) */
  public double c2get() {
    return c2.getAbsolutePosition();
  }
  /** returns cancoder angle in degrees [0, 360) */
  public double c3get() {
    return c3.getAbsolutePosition();
  }
  /** returns cancoder angle in degrees [0, 360) */
  public double c4get() {
    return c4.getAbsolutePosition();
  }

  /** returns continuous gyro angle in degrees, clockwise positive */
  public double gyroget() {
    SmartDashboard.putNumber("gyro", gyro.getYaw());
    return gyro.getYaw();
  }

  /** reset gyro [0,360) */
  public void gyroreset(double angle) {
    gyro.setYaw(angle);
  }


}
