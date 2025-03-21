package frc.robot.Utils;

import java.util.List;

import edu.wpi.first.math.util.Units;
import frc.robot.field.FieldConstants.AprilTagIDs;

public enum ReefTarget {
    Front(AprilTagIDs.getAllianceFrontReefId()),
    Back(AprilTagIDs.getAllianceBackReefId()),
    FrontLeft(AprilTagIDs.getAllianceFrontLeftReefId()),
    FrontRight(AprilTagIDs.getAllianceFrontRightReefId()),
    BackLeft(AprilTagIDs.getAllianceBackLeftReefId()),
    BackRight(AprilTagIDs.getAllianceBackRightReefId());
    public final int ApriltagId;
    private ReefTarget(int ApriltagId){
        this.ApriltagId = ApriltagId;
    }
}
