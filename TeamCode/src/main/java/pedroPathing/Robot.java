package pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

import pedroPathing.actions.MoveArm;
import pedroPathing.actions.MoveSlides;
import pedroPathing.constants.CalebConstants;

public class Robot {
    public boolean armZeroing = false;
    public boolean slidesZeroing = false;
    public boolean opMode;
    public ArrayList<Action> actions = new ArrayList<>();

    final Motors motors;
    final Servos servos;

    public Robot(HardwareMap hardwareMap, boolean reset) {
        motors = new Motors(hardwareMap);
        servos = new Servos(hardwareMap);

        motors.setDirection(DcMotorSimple.Direction.REVERSE);
        motors.left.setDirection(DcMotorSimple.Direction.FORWARD);

        if (reset) {
            motors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        motors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motors.rightArm.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
                new PIDFCoefficients(4, 0, 0.01, CalebConstants.FEEDFORWARD_ARM));
        opMode = true;
    }

    public void executeActionSequence() {
        for (int i = 0; i < actions.size(); i++) {
            if (actions.get(i).run() == ActionStatus.COMPLETED) {
                actions.remove(i);
                i--;
            }
        }
    }

    public void stop() {
        opMode = false;
    }

    public void checkResetSlideEncoders() {
        if (motors.left.getCurrentPosition() < 0 && (!motors.left.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION) || motors.left.getVelocity() < 10)) {
            resetSlideEncoders();
        }
    }
    public void resetSlideEncoders() {
        motors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftM2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftM2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void checkResetArmEncoder() {
        if (rightArm.getCurrentPosition() < 0) {
            resetArmEncoder();
        }
    }
    public void resetArmEncoder() {
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public class WaitForArm implements Action {
        int pos;
        int tolerance = -1;
        boolean up=false;
        public WaitForArm(int pos, int tolerance) {
            this.pos = pos;
            this.tolerance = tolerance;
        }
        public WaitForArm(int pos, boolean up) {
            this.pos = pos;
            this.up = up;
            tolerance=-1;
        }
        public boolean run() {
            int mult2 = 1;
            if (up) {
                mult2 = -1;
            }
            if (tolerance != -1 && Math.abs(rightArm.getCurrentPosition() - (int)(pos * mult)) < (int)(tolerance * mult)) {
                return false;
            } else if (tolerance == -1 && mult2 * (rightArm.getCurrentPosition() - (int)(pos * mult)) < 0) {
                return false;
            }
            return true;
        }
    }
    public Action waitForArm(int pos, int tolerance, boolean action) {
        Action a = new WaitForArm(pos, tolerance);
        if (!action) {
            while (opMode && !a.run()) {
                try {
                    Thread.sleep(SLEEPTIME);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
        }
        return a;
    }
    public Action waitForArm(int pos, boolean up, boolean action) {
        Action a = new WaitForArm(pos, up);
        if (!action) {
            while (opMode && !a.run()) {
                try {
                    Thread.sleep(SLEEPTIME);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
        }
        return a;
    }
    public Action waitForArm(boolean action) {
        return waitForArm(rightArm.getTargetPosition(), 50, action);
    }
    public class WaitForSlides implements Action {
        int pos;
        int tolerance = -1;
        boolean up=false;
        public WaitForSlides(int pos, int tolerance) {
            this.pos = pos;
            this.tolerance = tolerance;
        }
        public WaitForSlides(int pos, boolean up) {
            this.pos = pos;
            this.up = up;
            tolerance=-1;
        }
        public boolean run() {
            int mult2 = 1;
            if (up) {
                mult2 = -1;
            }
            if (tolerance != -1 && Math.abs(leftM.getCurrentPosition() - (int)(pos * mult)) < (int)(tolerance * mult)) {
                return false;
            } else if (tolerance == -1 && mult2 * (leftM.getCurrentPosition() - (int)(pos * mult)) < 0) {
                return false;
            }
            return true;
        }
    }

    public Action waitForSlides(int pos, int tolerance, boolean action) {
        Action a = new WaitForSlides(pos, tolerance);
        if (!action) {
            while (opMode && !a.run()) {
                try {
                    Thread.sleep(SLEEPTIME);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
        }
        return a;
    }
    public Action waitForSlides(int pos, boolean up, boolean action) {
        Action a = new WaitForSlides(pos, up);
        if (!action) {
            while (opMode && !a.run()) {
                try {
                    Thread.sleep(SLEEPTIME);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
        }
        return a;
    }
    public Action waitForSlides(boolean action) {
        return waitForSlides(leftM.getTargetPosition(), 100, action);
    }
    public Action zeroArm(boolean action) {
        Action a = new ZeroArm();
        if (!action) {
            while (opMode && !a.run()) {
                try {
                    Thread.sleep(SLEEPTIME);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
        }
        return a;
    }
    public class ZeroArm implements Action {
        int timesAtZero = 0;

        boolean init = false;
        @Override
        public boolean run() {
            if (!init) {
                armZeroing = true;
                rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightArm.setPower(-1);
                init = true;
            }
            if (opMode && timesAtZero < 3) {
                if (rightArm.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION) || !armZeroing) {
                    armZeroing = false;
                    return false;
                }
                if (Math.abs(rightArm.getVelocity()) < 50 && rightArm.getCurrentPosition() < 500) {
                    timesAtZero++;
                } else {
                    timesAtZero = 0;
                }
            } else {
                armZeroing = false;
                rightArm.setPower(0);
                if (opMode) {
                    resetArmEncoder();
                }
                return false;
            }
            return true;
        }
    }
    public Action zeroSlides(boolean action) {
        Action a = new ZeroSlides();
        if (!action) {
            while (opMode && !a.run()) {
                try {
                    Thread.sleep(SLEEPTIME);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
        }
        return a;
    }
    public class ZeroSlides implements Action {
        int timesAtZero = 0;
        double power = -0.7;
        int vel = 20;
        boolean init = false;

        @Override
        public boolean run() {
            if (!init) {
                leftM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftM2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftM.setPower(power);
                leftM2.setPower(power);
                rightM.setPower(power);
                slidesZeroing = true;
                init = true;
            }
            if (opMode && timesAtZero < 3) {
                if (leftM.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION) || !slidesZeroing) {
                    slidesZeroing = false;
                    return false;
                }
                if (Math.abs(leftM.getVelocity()) < vel && Math.abs(leftM2.getVelocity()) < vel && Math.abs(rightM.getVelocity()) < vel && leftM.getCurrentPosition() < 500) {
                    timesAtZero++;
                } else {
                    timesAtZero = 0;
                }
            } else {
                slidesZeroing = false;
                leftM.setPower(0);
                leftM2.setPower(0);
                rightM.setPower(0);
                if (opMode) {
                    resetSlideEncoders();
                }
                return false;
            }
            return true;
        }
    }
}