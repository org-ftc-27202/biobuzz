package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class Paths {
    private Alliance alliance;
    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose, launchPose, flowerPose;








    public Paths(Alliance alliance){
        this.alliance = alliance;
        if (alliance==  Alliance.BLUE_ALLIANCE){
        p.mirrorX(70.75);
        }
        startPose = p.of(MainTeleOp.RED_RESET_X, MainTeleOp.RED_RESET_Y, MainTeleOp.RED_RESET_HEADING_DEG);
        launchPose = p.of(56, 24, 90);
        flowerPose = p.of(48, 10, 180);





    }
    public Path toLaunch(){
        return line(startPose, launchPose).linear(startPose, launchPose);
    }
}
