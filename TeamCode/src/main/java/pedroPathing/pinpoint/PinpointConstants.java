package pedroPathing.pinpoint;//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

import com.acmerobotics.dashboard.config.Config;

import pedroPathing.pinpoint.GoBildaPinpointDriver.EncoderDirection;
import pedroPathing.pinpoint.GoBildaPinpointDriver.GoBildaOdometryPods;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class PinpointConstants {
    public static double forwardY = (double)1.0F;
    public static double strafeX = (double)-2.5F;
    public static DistanceUnit distanceUnit;
    public static String hardwareMapName;
    public static boolean useYawScalar;
    public static double yawScalar;
    public static boolean useCustomEncoderResolution;
    public static GoBildaPinpointDriver.GoBildaOdometryPods encoderResolution;
    public static double customEncoderResolution;
    public static GoBildaPinpointDriver.EncoderDirection forwardEncoderDirection;
    public static GoBildaPinpointDriver.EncoderDirection strafeEncoderDirection;

    static {
        distanceUnit = DistanceUnit.INCH;
        hardwareMapName = "pinpoint";
        useYawScalar = false;
        yawScalar = (double)1.0F;
        useCustomEncoderResolution = false;
        encoderResolution = GoBildaOdometryPods.goBILDA_4_BAR_POD;
        customEncoderResolution = 13.26291192;
        forwardEncoderDirection = EncoderDirection.REVERSED;
        strafeEncoderDirection = EncoderDirection.FORWARD;
    }
}
