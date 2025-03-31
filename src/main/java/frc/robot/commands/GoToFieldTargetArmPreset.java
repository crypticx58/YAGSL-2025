// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utils.ArmPreset;
import frc.robot.Utils.BargeTarget;
import frc.robot.Utils.CoralStationTarget;
import frc.robot.Utils.FieldTarget;
import frc.robot.Utils.ProcessorTarget;
import frc.robot.Utils.ReefTarget;
import frc.robot.field.FieldConstants.CoralStation;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToFieldTargetArmPreset extends Command {
  /** Creates a new GoToArmPreset. */
  private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
  private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  private FieldTarget fieldTarget;
  private ArmPreset armPreset;
  public GoToFieldTargetArmPreset() {
    //this.fieldTarget = fieldTarget;
    addRequirements(armSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    fieldTarget = swerveSubsystem.getCurrentFieldTarget();
    if (ReefTarget.isLowAlgaeReefTarget(fieldTarget)){
      armPreset = ArmPreset.LowAlgae;
    } else if (ReefTarget.isHighAlgaeReefTarget(fieldTarget)){
      armPreset = ArmPreset.HighAlgae;
    } else if (ReefTarget.isCoralReefTarget(fieldTarget)){
      armPreset = ArmPreset.L3;
    } else if (fieldTarget instanceof BargeTarget){
      armPreset = ArmPreset.BargeAlgae;
    } else if (fieldTarget instanceof ProcessorTarget){
      armPreset = ArmPreset.Processor;
    } else if (fieldTarget instanceof CoralStationTarget){
      armPreset = ArmPreset.CoralStationFeed;
    } 
  }

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
    return armSubsystem.isArmAtDesiredConfiguration(armPreset.armConfiguration, 3, Units.inchesToMeters(2), 5);
  }
}
