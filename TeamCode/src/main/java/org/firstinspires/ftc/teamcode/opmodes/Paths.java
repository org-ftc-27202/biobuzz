package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class Paths {
    private Alliance alliance;
    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose, launchPose, flowerPose, gardenPose;








    public Paths(Alliance alliance){
        this.alliance = alliance;
        if (alliance==  Alliance.BLUE_ALLIANCE){
        p.mirrorX(70.75);
        }
        startPose = p.of(MainTeleOp.RED_RESET_X, MainTeleOp.RED_RESET_Y, MainTeleOp.RED_RESET_HEADING_DEG);
        launchPose = p.of(56, 24, 90);
        flowerPose = p.of(10, 48, 180);
        gardenPose = p.of(10.5, MainTeleOp.RED_RESET_Y+2, 195);





    }
    public Path toLaunch(){
        return line(startPose, launchPose).linear(startPose, launchPose);
    }
    public Path toFlower(){
        return line(launchPose, flowerPose).linear(launchPose, flowerPose);
    }
    public Path toGarden(){
        return line(startPose, gardenPose).constant(gardenPose.heading()).with(
                Constants.foresightConfig.maxPathSpeed.at(0.28), // 60% of max achievable speed
                Constants.foresightConfig.maxVelocityConstraint.at(35.0));
    }
    public Path backFromGarden(){
        return line (gardenPose, startPose).constant(startPose.heading()).with(
                Constants.foresightConfig.maxPathSpeed.at(0.32), // 60% of max achievable speed
                Constants.foresightConfig.maxVelocityConstraint.at(35.0));
    }
}
