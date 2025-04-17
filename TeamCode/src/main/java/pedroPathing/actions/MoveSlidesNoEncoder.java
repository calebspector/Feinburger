package pedroPathing.actions;

import com.qualcomm.robotcore.hardware.DcMotor;

import pedroPathing.Action;
import pedroPathing.ActionStatus;
import pedroPathing.Motors;
import pedroPathing.Servos;
import pedroPathing.constants.CalebConstants;

public class MoveSlidesNoEncoder implements Action {
    Motors motors;
    Servos servos;
    double power;

    public MoveSlidesNoEncoder(Motors motors, Servos servos, double power) {
        this.motors = motors;
        this.servos = servos;
        this.power = power;
    }

    public ActionStatus run() {
        if (power >= 0 && motors.rightArm.getCurrentPosition() < 950) {
            int extraCush = 0;
            if (servos.shoulder.getPosition() > 0.8)
                extraCush = CalebConstants.armDownSlideOut;
            if (motors.left.getCurrentPosition() > (CalebConstants.slideMaxHoriz + extraCush) * CalebConstants.ARM_MULTIPLIER)
                power = 0;
            if (motors.left.getCurrentPosition() > (CalebConstants.slideMaxHoriz + CalebConstants.slideCushion + extraCush) * CalebConstants.ARM_MULTIPLIER)
                power = -.5;
        }
        if (motors.left.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION) || slidesZeroing) {
            if (Math.abs(power) > 0.5) {
                slidesZeroing = false;
                motors.right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.left2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                motors.left.setPower(power);
                motors.left2.setPower(power);
                motors.right.setPower(power);
            }
        } else {
            motors.left.setPower(power);
            motors.left2.setPower(power);
            motors.right.setPower(power);
        }

        return ActionStatus.COMPLETED;
    }
}
