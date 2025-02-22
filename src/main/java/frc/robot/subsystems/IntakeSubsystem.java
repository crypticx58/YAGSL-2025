// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.IntakeSensorsStates;
import frc.robot.utils.JointType;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private static IntakeSubsystem INSTANCE;

  @SuppressWarnings("WeakerAccess")
  public static IntakeSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new IntakeSubsystem();
    }
    return INSTANCE;
  }

  private final SparkMax IntakeMotor = new SparkMax(8, MotorType.kBrushless);

  private final DigitalInput entranceBreakBeamSensor = new DigitalInput(0);
  private final DigitalInput exitBreakBeamSensor = new DigitalInput(2);

  private IntakeSubsystem() {
    SparkMaxConfig IntakeMotorConfig = new SparkMaxConfig();
    IntakeMotorConfig.inverted(false).idleMode(IdleMode.kBrake);
    IntakeMotor.configure(IntakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public IntakeSensorsStates getIntakeSensorsStates() { //sensor.get() returns "False" when blocked
    return new IntakeSensorsStates(!entranceBreakBeamSensor.get(), !exitBreakBeamSensor.get());
  }

  public void setIntakeVelocity(double velocity){ // percentage from -1.0 to 1.0
    IntakeMotor.set(velocity);
  }
}
