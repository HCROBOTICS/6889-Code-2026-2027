package org.firstinspires.ftc.teamcode.pedro.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;

import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;


@Autonomous
public class ExampleAuto extends OpMode {
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees(); // (X, Y, heading in deg.)

    private final Pose startPose = p.of(24, 24, 0);
    private final Pose park = p.of(48, 48, 0);

    private final Pose controlPose1 = p.of(36, 60, 45);


    private Path park() {
        return curve(startPose, controlPose1, park).linear(startPose,park);
    }

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
    }

    @Override
    public void start() {
        schedule(follow(follower, park()));
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
    }
}
