package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.utils.ArmSpeeds;
import frc.robot.utils.JointType;
import frc.robot.utils.InputsManager.ForwardKinematicsInputsManager;


public class ArmControllerCommand extends Command {
    private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private final ForwardKinematicsInputsManager forwardKinematicsInputsManager;
    public ArmControllerCommand(ForwardKinematicsInputsManager forwardKinematicsInputsManager) {
        this.forwardKinematicsInputsManager = forwardKinematicsInputsManager;
        addRequirements(this.armSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        ArmSpeeds armSpeeds = forwardKinematicsInputsManager.getArmSpeeds();
        armSubsystem.setJointSpeed(JointType.Shoulder, armSpeeds.ShoulderSpeedDPS);
        armSubsystem.setJointSpeed(JointType.Telescopic, armSpeeds.TelescopicSpeedMPS);
        armSubsystem.setJointSpeed(JointType.Wrist, armSpeeds.WristSpeedDPS);
        armSubsystem.setJointSpeed(JointType.Intake, armSpeeds.IntakeSpeedPercent);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.stopJointsInPlace();
    }
}