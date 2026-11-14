package org.firstinspires.ftc.teamcode.pedro.teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "26-27 teleop (java)")
@Disabled
public class _2627teleop extends LinearOpMode {

    private DcMotor FL;
    private DcMotor BL;
    private DcMotor FR;
    private DcMotor BR;
    private Servo rightintakeservo;
    private Servo motorstopperservo;
    private Servo leftintakeservo;
    private DcMotor shooter;
    private DcMotor intake;

    int frontLeftPower;
    int frontRightPower;
    int backLeftPower;
    int backRightPower;

    /**
     * This OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
     * This code will work with either a Mecanum-Drive or an X-Drive train.
     * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
     *
     * Also note that it is critical to set the correct rotation direction for each motor. See details below.
     *
     * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
     * Each motion axis is controlled by one Joystick axis.
     *
     * 1) Axial -- Driving forward and backward -- Left-joystick Forward/Backward
     * 2) Lateral -- Strafing right and left -- Left-joystick Right and Left
     * 3) Yaw -- Rotating Clockwise and counter clockwise -- Right-joystick Right and Left
     *
     * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
     * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
     * the direction of all 4 motors (see code below).
     */
    @Override
    public void runOpMode() {
        int right_servo_pos;
        int left_servo_pos;
        double boost;
        ElapsedTime runtime;
        int motor_stopper;
        float axial;
        float lateral;
        float yaw;

        FL = hardwareMap.get(DcMotor.class, "FL");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");
        rightintakeservo = hardwareMap.get(Servo.class, "right intake servo");
        motorstopperservo = hardwareMap.get(Servo.class, "motor stopper servo");
        leftintakeservo = hardwareMap.get(Servo.class, "left intake servo");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");

        runtime = new ElapsedTime();
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);
        rightintakeservo.setDirection(Servo.Direction.REVERSE);
        motorstopperservo.setDirection(Servo.Direction.FORWARD);
        motor_stopper = 0;
        telemetry.addData("Status", "Initialized");
        right_servo_pos = 0;
        left_servo_pos = 0;
        rightintakeservo.setPosition(right_servo_pos);
        leftintakeservo.setPosition(left_servo_pos);
        telemetry.update();
        waitForStart();
        runtime.reset();
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        while (opModeIsActive()) {
            axial = -gamepad1.left_stick_y;
            lateral = gamepad1.left_stick_x;
            yaw = gamepad1.right_stick_x;
            if (gamepad1.left_bumper) {
                boost = -0.3;
            } else if (gamepad1.right_bumper) {
                boost = 0.2;
            } else {
                boost = 0;
            }
            if (gamepad1.left_stick_x > 0.07 || gamepad1.right_stick_x > 0.07 || gamepad1.left_stick_y > 0.07 || gamepad1.left_stick_x < -0.07 || gamepad1.right_stick_x < -0.07 || gamepad1.left_stick_y < -0.07) {
                frontLeftPower = (int) ((axial + lateral + yaw) * 0.7 + boost);
                frontRightPower = (int) (((axial - lateral) - yaw) * 0.7 + boost);
                backLeftPower = (int) (((axial - lateral) + yaw) * 0.7 + boost);
                backRightPower = (int) (((axial + lateral) - yaw) * 0.7 + boost);
            } else {
                frontLeftPower = 0;
                backRightPower = 0;
                frontRightPower = 0;
                backLeftPower = 0;
            }
            if (gamepad1.x) {
                intake.setPower(-0.4);
            } else if (gamepad1.y) {
                intake.setPower(0.4);
            } else {
                intake.setPower(0);
            }
            if (gamepad1.a && motor_stopper == 0) {
                motor_stopper = 1;
            } else if (gamepad1.b && motor_stopper == 1) {
                motor_stopper = 0;
            }
            if (motor_stopper == 0) {
                motorstopperservo.setPosition(0);
            } else if (motor_stopper == 1) {
                motorstopperservo.setPosition(0.25);
            }
            if (gamepad1.dpad_up) {
                while (!!gamepad1.dpad_up) {
                    rightintakeservo.setPosition(0.45);
                    leftintakeservo.setPosition(0.3);
                    sleep(150);
                    rightintakeservo.setPosition(0.9);
                    leftintakeservo.setPosition(0.6);
                    sleep(150);
                }
            }
            if (gamepad1.dpad_down) {
                rightintakeservo.setPosition(0.4);
                leftintakeservo.setPosition(0.2);
            }
            if (gamepad1.right_trigger > 0) {
                shooter.setPower(-0.55);
            } else {
                shooter.setPower(0);
            }
            FL.setPower(frontLeftPower);
            FR.setPower(frontRightPower);
            BL.setPower(backLeftPower);
            BR.setPower(backRightPower);
            telemetry.addData("right servo pos", right_servo_pos);
            telemetry.addData("shooter power", gamepad1.right_trigger);
            telemetry.update();
        }
    }

    /**
     * This function is used to test your motor directions.
     *
     * Each button should make the corresponding motor run FORWARD.
     *
     *   1) First get all the motors to take to correct positions on the robot
     *      by adjusting your Robot Configuration if necessary.
     *
     *   2) Then make sure they run in the correct direction by modifying the
     *      the setDirection() calls above.
     */
    private void testMotorDirections() {
        frontLeftPower = gamepad1.x ? 1 : 0;
        backLeftPower = gamepad1.a ? 1 : 0;
        frontRightPower = gamepad1.y ? 1 : 0;
        backRightPower = gamepad1.b ? 1 : 0;
    }
}