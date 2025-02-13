package frc.robot.Utils;

import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

import frc.robot.Constants;

public class InputsManager {
    public static double applyDeadBand(double value, double deadband){
        return Math.abs(value)<=deadband?0f:value;
    }
    public static final class ForwardKinematicsInputsManager {
        private final Supplier<Double> baseSpeedFunc, shoulderSpeedFunc, elbowSpeedFunc, wristSpeedFunc;
        public ForwardKinematicsInputsManager(Supplier<Double> baseSpeedFunc, Supplier<Double> shoulderSpeedFunc,
                                   Supplier<Double> elbowSpeedFunc, Supplier<Double> wristSpeedFunc){
            this.baseSpeedFunc = baseSpeedFunc;
            this.shoulderSpeedFunc = shoulderSpeedFunc;
            this.elbowSpeedFunc = elbowSpeedFunc;
            this.wristSpeedFunc = wristSpeedFunc;
        }
        public ArmSpeeds getArmSpeeds(){
            List<Double> armSpeeds = Arrays.asList(baseSpeedFunc.get(), shoulderSpeedFunc.get(), elbowSpeedFunc.get(), wristSpeedFunc.get())
                            .stream()
                            .map(x->applyDeadBand(x, Constants.OperatorConstants.ARM_SPEEDS_DEADBAND))
                            .toList();
            // double baseSpeed = applyDeadBand(baseSpeedFunc.get(), 0.3);
            // double elbowSpeed = applyDeadBand(elbowSpeedFunc.get(), 0.3);
            //return new ArmSpeeds(baseSpeed, elbowSpeed, 0,0);
            return new ArmSpeeds(armSpeeds.get(0)*JointType.Shoulder.maxSpeed, armSpeeds.get(1)*JointType.Telescopic.maxSpeed, armSpeeds.get(2)*JointType.Wrist.maxSpeed, armSpeeds.get(3)*JointType.Intake.maxSpeed);
        }
    }
}
