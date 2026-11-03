package org.firstinspires.ftc.teamcode.pedro.autos;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;

import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.qualcomm.hardware.limelightvision.*;

import java.util.List;

@Autonomous
public class PedroTest extends OpMode {
    private Follower follower;
    Limelight3A limelight;
    private final PoseFactory p = PoseFactory.degrees(); // (X, Y, heading in deg.)

    private final Pose startPose = p.of(8, 24, 180);
    private final Pose path1 = p.of(61.07, 24, 180);
    private final Pose path2 = p.of(10, 10, 180);
    private final Pose path3 = p.of(8, 10, 180);
    private final Pose path4 = p.of(54.87, 24.00, 90);

    private final Pose controlPose1 = p.of(58.69591527987897, 8.888048411497719, 0);


    private Path path1() {
        return line(startPose, path1).linear(startPose,path1).with(Constants.foresightConfig.maxPathSpeed.at(0.75));
    }
    private Path path2() {
        return curve(path1, controlPose1, path2).linear(path1, path2).with(Constants.foresightConfig.maxPathSpeed.at(0.5));
    }
    private Path path3() {
        return line(path2, path3).linear(path2, path3).with(Constants.foresightConfig.maxPathSpeed.at(0.4));
    }
    private Path path4() {
        return line(path3, path4).linear(path3, path4).with(Constants.foresightConfig.maxPathSpeed.at(0.75));
    }

    private Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                waitMs(200),
                // Add mechanisms here.
                follow(follower, path2()),
                waitMs(3000),
                follow(follower, path3()),
                waitMs(1000),
                follow(follower, path4())
        );
    }


    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(1);
        //result.getPipeLineIndex()

        //Servo leftServo = hardwareMap.get(Servo.class, "left servo");
        //Servo rightServo = hardwareMap.get(Servo.class, "right servo");
        //DcMotor intake = hardwareMap.get(DcMotor.class, "intake");

        //leftServo.setPosition(0);
    }

    @Override
    public void start() {
        //schedule(autoRoutine());        //uncomment this if not testing limelight
    }

    @Override
    public void loop() {
        //follower.update();              //uncomment this if not testing limelight

        LLResult result = limelight.getLatestResult();

        telemetry.addData("Limelight Connected", limelight.isConnected());

        if (result != null && result.isValid()) {

            telemetry.addData("Pipeline", result.getPipelineIndex());

            List<LLResultTypes.FiducialResult> fiducials =
                    result.getFiducialResults();

            telemetry.addData("AprilTags Detected", fiducials.size());

            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                telemetry.addData(
                        "Tag ID",
                        fiducial.getFiducialId()
                );

                telemetry.addData(
                        "Target X",
                        fiducial.getTargetXDegrees()
                );

                telemetry.addData(
                        "Target Y",
                        fiducial.getTargetYDegrees()
                );
            }

        } else {
            telemetry.addLine("No valid Limelight result");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}