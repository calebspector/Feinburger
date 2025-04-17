package pedroPathing.constants;

import com.pedropathing.localization.Encoder;
import com.pedropathing.localization.constants.ThreeWheelConstants;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = .001989436789;
        ThreeWheelConstants.strafeTicksToInches = .001989436789;
        ThreeWheelConstants.turnTicksToInches = .001989436789;
        ThreeWheelConstants.leftY = 6.22484252;
        ThreeWheelConstants.rightY = -6.22484252;
        ThreeWheelConstants.strafeX = 3.75799213;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "fl";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "bl";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "fr";
        ThreeWheelConstants.leftEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.REVERSE;
    }
}




