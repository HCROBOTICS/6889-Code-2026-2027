package org.firstinspires.ftc.teamcode.pedro.autos;

import com.pedropathing.ivy.Command;
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
import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class park extends OpMode {
    private Follower follower;
    private Servo leftServo;
    private Servo rightServo;
    private Servo motorstopperservo;
    private DcMotor shooter;
    private DcMotor intake;
    //int rightStartServo;
    //int leftStartServo;
    Limelight3A limelight;
    ArrayList<Integer> tags = new ArrayList<>();
    private final PoseFactory p = PoseFactory.degrees(); // (X, Y, heading in deg.)

    private final Pose startPose = p.of(8, 24, 180);
    private final Pose path1 = p.of(18.30, 24.00, 180);
    private final Pose path2 = p.of(11.06, 127.31, 180);
    private final Pose controlPose1 = p.of(27.453857791225424, 54.282904689863834, 0);


    private Path path1() {
        return line(startPose, path1).linear(startPose, path1).with(Constants.foresightConfig.maxPathSpeed.at(0.75));
    }
    private Path path2() {
        return curve(path1, controlPose1, path2).linear(path1, path2).with(Constants.foresightConfig.maxPathSpeed.at(0.75));
    }

    private boolean hasAnyTag(int... tagIds) {
        for (int tagId : tagIds) {
            if (tags.contains(tagId)) {
                return true;
            }
        }
        return false;
    }

    private Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2())
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
        limelight.pipelineSwitch(7); //Red is 6, Blue is 7
        //result.getPipeLineIndex()


        leftServo = hardwareMap.get(Servo.class, "left intake servo");
        rightServo = hardwareMap.get(Servo.class, "right intake servo");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        motorstopperservo = hardwareMap.get(Servo.class, "motor stopper servo");

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightServo.setDirection(Servo.Direction.REVERSE);

        //rightStartServo = rightServo.getPosition();
        //leftStartServo = rightServo.getPosition();
        //leftServo.setPosition(0);

        telemetry.addData("Status:", "Press start");
        telemetry.update();

    }

    @Override
    public void start() {
        schedule(autoRoutine());        //comment this if not using paths
        intake.setPower(0);
        shooter.setPower(-0.3);
    }

    @Override
    public void loop() {

        rightServo.setPosition(0.35);
        leftServo.setPosition(0.175);
        // Update PedroPathing
        follower.update();

        // IMPORTANT: Execute the Ivy scheduler every loop!
        Scheduler.execute();

        // Clear old detections so invalid frames don't leave stale tags
        tags.clear();

        LLResult result = limelight.getLatestResult();

        telemetry.addData("Limelight Connected", limelight.isConnected());
        telemetry.addData("Following Path", follower.following());

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

                tags.add(fiducial.getFiducialId());
            }

        } else {
            telemetry.addLine("No valid Limelight result");
        }

        telemetry.addData("Detected Tag IDs", tags.toString());

        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }

}