// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestSubsystem extends SubsystemBase {

  SparkMax motor = new SparkMax(10, MotorType.kBrushless);
  SparkMaxConfig motorConfig = new SparkMaxConfig();
  AbsoluteEncoder absoluteEncoder = motor.getAbsoluteEncoder();
  /** Creates a new TestSubsystem. */
  public TestSubsystem() {
    motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Abs Encoder", absoluteEncoder.getPosition());
    // This method will be called once per scheduler run
  }
}
