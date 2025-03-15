// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utils.ArmPreset;
import frc.robot.subsystems.ArmSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToArmPreset extends Command {
  /** Creates a new GoToArmPreset. */
  private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
  private final ArmPreset armPreset;
  public GoToArmPreset(ArmPreset armPreset) {
    this.armPreset = armPreset;
    addRequirements(armSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.setArmConfiguration(armPreset.armConfiguration);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armSubsystem.isArmAtDesiredConfiguration(armPreset.armConfiguration, 0.5, Units.inchesToMeters(0.2), 0.5);
  }
}
