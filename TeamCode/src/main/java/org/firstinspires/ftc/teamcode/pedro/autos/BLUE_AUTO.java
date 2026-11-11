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
public class BLUE_AUTO extends OpMode {
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
    private final Pose checkPath1 = p.of(58.82, 37.17, 90);
    private final Pose path1r = p.of(19, 20, 45);
    private final Pose path1l = p.of(58.82, 122.90, -90);
    private final Pose path1lDone = p.of(12.37, 106.83, -180);
    private final Pose path1rDone = p.of(12.37, 90, 180); //curve with ctrlPose1

    private final Pose controlPose1 = p.of(13.72919818456884, 26.615733736762472, 0);


    private Path checkPath() {
        return line(startPose, checkPath1).linear(startPose, checkPath1).with(Constants.foresightConfig.maxPathSpeed.at(0.75));
    }
    private Path path1r() {
        return line(checkPath1, path1r).linear(checkPath1, path1r).with(Constants.foresightConfig.maxPathSpeed.at(0.75));
    }
    private Path path1l() {
        return line(checkPath1, path1l).constant(path1l).with(Constants.foresightConfig.maxPathSpeed.at(0.75));
    }
    private Path path1lDone() {
        return line(path1l, path1lDone).constant(path1lDone).with(Constants.foresightConfig.maxPathSpeed.at(0.75));
    }

    private Path path1rDone() {
        return curve(path1r, controlPose1, path1rDone).linear(path1r, path1rDone).with(Constants.foresightConfig.maxPathSpeed.at(0.75));
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
                // Drive to AprilTag checking position
                follow(follower, checkPath()),

                waitMs(200),

                // If ANY are detected:
                //      RIGHT
                // Otherwise:
                //      LEFT
                conditional(
                        () -> hasAnyTag(42, 43, 44, 45),

                        // RIGHT
                        sequential(
                                follow(follower, path1r()),
                                shoot(),
                                follow(follower, path1rDone())
                        ),

                        // LEFT
                        sequential(
                                follow(follower, path1l()),
                                shoot(),
                                follow(follower, path1lDone())
                        )
                )
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
        limelight.pipelineSwitch(7);
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

    private Command shoot() {
        return sequential(
                // Start shooter
                instant(() -> shooter.setPower(-0.75)),

                // Let shooter spin up
                waitMs(2000),

                // Open the stopper
                instant(() -> motorstopperservo.setPosition(0.3)),

                // Feed the balls
                instant(() -> intake.setPower(-0.9)),

                // Keep feeding for 1 second
                waitMs(1000),

                // Stop feeding
                instant(() -> intake.setPower(0)),

                // Close stopper
                instant(() -> motorstopperservo.setPosition(0)),

                // Stop shooter
                instant(() -> shooter.setPower(0))
        );
    }
}