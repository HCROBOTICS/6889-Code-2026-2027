package org.firstinspires.ftc.teamcode.pedro.autos;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

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
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
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
        limelight.pipelineSwitch(6);
        aprilTagProcessor = new AprilTagProcessor.Builder().build();
        //result.getPipeLineIndex()

        Servo leftServo = hardwareMap.get(Servo.class, "left servo");
        Servo rightServo = hardwareMap.get(Servo.class, "right servo");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();

        //leftServo.setPosition(0);
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials =
                    result.getFiducialResults();

            if (!fiducials.isEmpty()) {
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    telemetry.addData("Detected Tag ID", fiducial.getFiducialId());
                }
            } else {
                telemetry.addLine("No AprilTags detected by Limelight");
            }
        } else {
            telemetry.addLine("No valid Limelight result");
        }

        telemetry.update();
    }
}