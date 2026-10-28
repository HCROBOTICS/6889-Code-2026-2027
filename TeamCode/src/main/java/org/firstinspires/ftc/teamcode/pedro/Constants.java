package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("FL");
        c.frontRightName.set("FR");
        c.backLeftName.set("BL");
        c.backRightName.set("BR");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-0.8783022812970981);
        c.yPodOffset.set(-0.8965787361925982);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.18509632538502246);
                Controller secondaryTranslationalForward = Controller.proportional(0.06838814636486222);
                Controller primaryTranslationalLateral = Controller.proportional(0.23319466439309308);
                Controller secondaryTranslationalLateral = Controller.proportional(0.08615919741705594);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.011448029496438427));
                c.brake.set(Controller.proportionalFeedforward(0.009730825071972662));

                c.headingFeedback.set(Controller.proportional(3.762408530582031));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.055634190760177844, 0.00515541648733341));

                c.linearBrakeCoefficients.set(Matrix.diag(0.058925391308790466, 0.039950439360472705));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001686796518961263, 0.0020034717560337804));

                c.maxAchievableForwardVelocity.set(82.27185386157927);
                c.maxAchievableStrafeVelocity.set(69.65682909055397);
                c.naturalForwardDeceleration.set(28.55671291534947);
                c.naturalStrafeDeceleration.set(50.80580421768067);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}