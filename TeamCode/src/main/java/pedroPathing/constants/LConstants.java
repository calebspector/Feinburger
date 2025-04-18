package pedroPathing.constants;

import com.pedropathing.localization.Encoder;
import com.pedropathing.localization.constants.ThreeWheelConstants;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = 0.001966727105338751;
        ThreeWheelConstants.strafeTicksToInches = 0.0019737725879494564;
        ThreeWheelConstants.turnTicksToInches = 0.001980797602550995;
        ThreeWheelConstants.leftY = 6.22484252;
        ThreeWheelConstants.rightY = -6.22484252;
        ThreeWheelConstants.strafeX = 3.75799213;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "fl";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "bl";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "fr";
        ThreeWheelConstants.leftEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.rightEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.REVERSE;
    }
}




