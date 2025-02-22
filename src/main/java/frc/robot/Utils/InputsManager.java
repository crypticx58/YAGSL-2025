package frc.robot.utils;

import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

import frc.robot.Constants;

public class InputsManager {
    public static double applyDeadBand(double value, double deadband){
        return Math.abs(value)<=deadband?0f:value;
    }
    public static final class ForwardKinematicsInputsManager {
        private final Supplier<Double> shoulderSpeedFunc, telescopicSpeedFunc, wristSpeedFunc;
        public ForwardKinematicsInputsManager(Supplier<Double> shoulderSpeedFunc, Supplier<Double> telescopicSpeedFunc,
                                   Supplier<Double> wristSpeedFunc){
            this.shoulderSpeedFunc = shoulderSpeedFunc;
            this.telescopicSpeedFunc = telescopicSpeedFunc;
            this.wristSpeedFunc = wristSpeedFunc;
        }
        public ArmSpeeds getArmSpeeds(){
            List<Double> armSpeeds = Arrays.asList(shoulderSpeedFunc.get(), telescopicSpeedFunc.get(), wristSpeedFunc.get())
                            .stream()
                            .map(x->applyDeadBand(x, Constants.OperatorConstants.ARM_SPEEDS_DEADBAND))
                            .toList();
            return new ArmSpeeds(armSpeeds.get(0)*JointType.Shoulder.maxSpeed, armSpeeds.get(1)*JointType.Telescopic.maxSpeed, armSpeeds.get(2)*JointType.Wrist.maxSpeed);
        }
    }
}
