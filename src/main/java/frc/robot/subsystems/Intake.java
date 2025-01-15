// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private SparkMax IntakeIn = new SparkMax(Constants.IntakeID, MotorType.kBrushless);
  private double voltage = 0.0;
  /** Creates a new Intake. */
  public Intake() {
    
  }

  public void setMoterVoltage(double voltage){
    this.voltage = voltage;
  }

  @Override
  public void periodic() {
    IntakeIn.set(voltage);
    System.out.println(voltage);
  }
}
