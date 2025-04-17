package pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

public class Robot{
    public static final double SWEEPDOWN=0;
    public static final double SWEEPUP = 1;
    public static final double UPSCORE = .95;
    public static final double UPDOWN = 0.14;
    public static final double HANGCLOSE = 0.3;
    public static final double HANGOPEN = 0.8;
    public static final double HANGMID = 0.4;
    public static final double CLAWOPEN = 0.825;
    public static final double CLAWCLOSE = 0.28;
    public static final double CLAWLOOSE = CLAWCLOSE+0.07;
    public static final double WRISTNORMAL = 0.55;
    public static final double degToWrist = WRISTNORMAL/180;
    public static final double SHOULDERSCORE = 0.94;
    public static final double SHOULDERPICKUP = 0.04;
    public static final double SHOULDERBASKET = 0.22;
    public static final double SHOULDERMID = 0.65;
    public static final double ACTUATORENGAGE = 0.7;
    public static final double ACTUATORDISENGAGE = 0.45;
    public static final int slideMaxHoriz = 1050;
    public static final int slideCushion = 150;
    public static final int armDownSlideOut = 350;
    public static final int armMax=2000;
    public static final int armWithShoulderInFront=2200;
    public static final int armCushion=100;
    public static int motorRPM = 435;
    public static int slideTicksToInch=78;
    public static double mult=435.0/(double)motorRPM;
    //public static double armToDeg = 360.0/8192.0;
    public final DcMotorEx leftM;
    public final DcMotorEx leftM2;
    public final DcMotorEx rightM;
    public final DcMotorEx rightArm;
    public final Servo sweep;
    public final Servo up;
    public final Servo hang;
    public final Servo wrist;
    public final Servo shoulder;
    public final Servo shoulder2;
    public final Servo claw;
    public final Servo actuator;
    public final Servo pullUp;
    //public final CameraName arducam;
    //public ColorBlobLocatorProcessor colorLocator;
    public boolean armZeroing=false;
    public boolean slidesZeroing=false;
    public boolean opMode;
    /** @ noinspection deprecation*/ //public VisionPortal vp;
    public ArrayList<Action> actions = new ArrayList<>();
    public Robot(HardwareMap hardwareMap,boolean reset) {
        leftM = hardwareMap.get(DcMotorEx.class, "leftM");
        leftM2 = hardwareMap.get(DcMotorEx.class, "leftM2");
        rightM = hardwareMap.get(DcMotorEx.class, "rightM");
        rightArm = hardwareMap.get(DcMotorEx.class, "rightArm");

        leftM2.setDirection(DcMotorSimple.Direction.REVERSE);
        rightM.setDirection(DcMotorSimple.Direction.REVERSE);
        rightArm.setDirection(DcMotorSimple.Direction.REVERSE);

        if (reset) {
            leftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftM2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        rightM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftM2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftM2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sweep = hardwareMap.get(Servo.class, "sweep");
        up = hardwareMap.get(Servo.class, "up");
        hang = hardwareMap.get(Servo.class, "hang");
        wrist = hardwareMap.get(Servo.class, "wrist");
        shoulder = hardwareMap.get(Servo.class, "shoulder");
        shoulder2 = hardwareMap.get(Servo.class, "shoulder2");
        claw = hardwareMap.get(Servo.class, "claw");
        actuator = hardwareMap.get(Servo.class, "actuator");
        pullUp = hardwareMap.get(Servo.class, "pullUp");

        PIDCoefficients arm = new PIDCoefficients(4, 0, 0.01);
        rightArm.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, arm);
        opMode=true;
    }
    public void doLotsOfGoodThings(){
        for (int i=0;i<actions.size();i++){
            if (!actions.get(i).run()){
                actions.remove(i);
                i--;
            }
        }
    }
    public void stop(){
        opMode=false;
    }
    public class MoveArm implements Action{
        int position;
        double power;
        public MoveArm(int pos,double pow){
            position=pos;
            power=pow;
        }
        @Override
        public boolean run() {
            rightArm.setTargetPosition(position);
            rightArm.setTargetPositionTolerance(30);
            rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightArm.setPower(power);
            return false;
        }
    }
    public Action moveArm(int position,boolean action){
        return moveArm(position,1,action);
    }
    public Action moveArm(int position, double power,boolean action){
        Action a = new MoveArm(position,power);
        if (!action){
            a.run();
        }
        return a;
    }

    public class MoveSlides implements Action{
        int position;
        public MoveSlides(int pos){
            position=pos;
        }
        @Override
        public boolean run() {
            leftM.setTargetPosition((int)(position*mult));
            leftM2.setTargetPosition((int)(position*mult));
            rightM.setTargetPosition((int)(position*mult));

            leftM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftM2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftM.setPower(1);
            leftM2.setPower(1);
            rightM.setPower(1);
            return false;
        }
    }
    public Action moveSlides(int position,boolean action){
        Action a = new MoveSlides(position);
        if (!action){
            a.run();
        }
        return a;
    }
    public void moveSlidesNoEncoder(double power){
        if (power>=0&&rightArm.getCurrentPosition()<950){
            int extraCush=0;
            if (shoulder.getPosition()>0.8)
                extraCush = armDownSlideOut;
            if (leftM.getCurrentPosition()>(slideMaxHoriz+extraCush)*mult)
                power=0;
            if (leftM.getCurrentPosition()>(slideMaxHoriz+slideCushion+extraCush)*mult)
                power=-.5;
        }
        if (leftM.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION)||slidesZeroing){
            if (Math.abs(power)>0.5){
                slidesZeroing=false;
                rightM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftM2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                leftM.setPower(power);
                leftM2.setPower(power);
                rightM.setPower(power);
            }
        }
        else{
            leftM.setPower(power);
            leftM2.setPower(power);
            rightM.setPower(power);
        }
    }
    public void moveArmNoEncoder(double power){
        if (rightArm.getCurrentPosition()>armMax+armCushion&&shoulder.getPosition()<0.9||(shoulder.getPosition()>0.9&&rightArm.getCurrentPosition()>armWithShoulderInFront+armCushion)){
            power=Math.min(power,-0.5);
        }
        else if (rightArm.getCurrentPosition()>armMax&&shoulder.getPosition()<0.9||(shoulder.getPosition()>0.9&&rightArm.getCurrentPosition()>armWithShoulderInFront)){
            power=Math.min(power,0);
        }

        if (rightArm.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION)||armZeroing){
            if (Math.abs(power)>0.5/*||(!armZeroing&&Math.abs(rightArm.getCurrentPosition()-rightArm.getTargetPosition())<50)*/){
                armZeroing=false;
                rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightArm.setPower(power);
            }
        }
        else{
            rightArm.setPower(power);
        }
    }
    public class MoveSweep implements Action{
        double position;
        public MoveSweep(double pos){
            position=pos;
        }
        @Override
        public boolean run() {
            sweep.setPosition(position);
            return false;
        }
    }
    public Action moveSweep(double pos,boolean action){
        Action a = new MoveSweep(pos);
        if (!action){
            a.run();
        }
        return a;
    }
    public class MoveUp implements Action{
        double position;
        public MoveUp(double pos){
            position=pos;
        }
        @Override
        public boolean run() {
            up.setPosition(position);
            return false;
        }
    }
    public Action moveUp(double pos,boolean action){
        Action a = new MoveUp(pos);
        if (!action){
            a.run();
        }
        return a;
    }
    public class MoveHang implements Action{
        double position;
        public MoveHang(double pos){
            position=pos;
        }
        @Override
        public boolean run() {
            hang.setPosition(position);
            return false;
        }
    }
    public Action moveHang(double pos, boolean action){
        Action a = new MoveHang(pos);
        if (!action){
            a.run();
        }
        return a;
    }
    public class MoveClaw implements Action{
        double position;
        public MoveClaw(double pos){
            position=pos;
        }
        @Override
        public boolean run() {
            claw.setPosition(position);
            return false;
        }
    }
    public Action moveClaw(double pos, boolean action){
        Action a = new MoveClaw(pos);
        if (!action){
            a.run();
        }
        return a;
    }
    public class MoveShoulder implements Action{
        double position;
        public MoveShoulder(double pos){
            position=pos;
        }
        @Override
        public boolean run() {
            shoulder.setPosition(position);
            shoulder2.setPosition(position);
            return false;
        }
    }
    public Action moveShoulder(double pos,boolean action){
        Action a = new MoveShoulder(pos);
        if (!action){
            a.run();
        }
        return a;
    }
    public class MoveWrist implements Action{
        double position;
        public MoveWrist(double pos){
            position=pos;
        }
        @Override
        public boolean run() {
            wrist.setPosition(position);
            return false;
        }
    }
    public Action moveWrist(double pos, boolean action){
        Action a = new MoveWrist(pos);
        if (!action){
            a.run();
        }
        return a;
    }
    public class MoveActuator implements Action{
        double position;
        double pullUpPower;
        public MoveActuator(double pos,double pow){
            position=pos;
            pullUpPower=pow;
        }
        @Override
        public boolean run() {
            actuator.setPosition(position);
            pullUp.setPosition(pullUpPower);
            return false;
        }
    }
    public Action disengageActuator(boolean action){
        Action a = new MoveActuator(ACTUATORDISENGAGE,0.5);
        if (!action){
            a.run();
        }
        return a;
    }
    public Action engageActuator(boolean action){
        Action a = new MoveActuator(ACTUATORENGAGE,1);
        if (!action){
            a.run();
        }
        return a;
    }

    public void checkResetSlideEncoders(){
        if (leftM.getCurrentPosition()<0&&(!leftM.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION)||leftM.getVelocity()<10)){
            resetSlideEncoders();
        }
    }
    public void resetSlideEncoders(){
        rightM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftM2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftM2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void checkResetArmEncoder(){
        if (rightArm.getCurrentPosition()<0){
            resetArmEncoder();
        }
    }
    public void resetArmEncoder(){
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public class WaitForArm implements Action{
        int pos;
        int tolerance=-1;
        boolean up;
        public WaitForArm(int pos, int tolerance){
            this.pos=pos;
            this.tolerance=tolerance;
        }
        public WaitForArm(int pos, boolean up){
            this.pos=pos;
            this.up=up;
        }
        public boolean run(){
            int mult2 = 1;
            if (up){
                mult2=-1;
            }
            if (tolerance!=-1&&!(opMode&&Math.abs(rightArm.getCurrentPosition()-(int)(pos*mult))>(int)(tolerance*mult))){
                return false;
            }
            else if (tolerance==-1&&!(opMode&&mult2*(rightArm.getCurrentPosition()-(int)(pos*mult))> 0)){
                return false;
            }
            return true;
        }
    }
    public Action waitForArm(int pos, int tolerance, boolean action){
        Action a = new WaitForArm(pos,tolerance);
        if (!action){
            while (opMode&&!a.run()){
                try{
                    Thread.sleep(80);
                }catch(InterruptedException e){
                    throw new RuntimeException(e);
                }
            }
        }
        return a;
    }
    public Action waitForArm(int pos, boolean up,boolean action){
        Action a = new WaitForArm(pos,up);
        if (!action){
            while (opMode&&!a.run()){
                try{
                    Thread.sleep(80);
                }catch(InterruptedException e){
                    throw new RuntimeException(e);
                }
            }
        }
        return a;
    }
    public Action waitForArm(boolean action){
        return waitForArm(rightArm.getTargetPosition(),50,action);
    }
    public class WaitForSlides implements Action{
        int pos;
        int tolerance=-1;
        boolean up;
        public WaitForSlides(int pos, int tolerance){
            this.pos=pos;
            this.tolerance=tolerance;
        }
        public WaitForSlides(int pos, boolean up){
            this.pos=pos;
            this.up=up;
        }
        public boolean run(){
            int mult2 = 1;
            if (up){
                mult2=-1;
            }
            if (tolerance!=-1&&!(opMode&&Math.abs(rightArm.getCurrentPosition()-(int)(pos*mult))>(int)(tolerance*mult))){
                return false;
            }
            else if (tolerance==-1&&!(opMode&&mult2*(rightArm.getCurrentPosition()-(int)(pos*mult))> 0)){
                return false;
            }
            return true;
        }
    }

    public Action waitForSlides(int pos,int tolerance,boolean action){
        Action a = new WaitForSlides(pos,tolerance);
        if (!action){
            while (opMode&&!a.run()){
                try{
                    Thread.sleep(80);
                }catch(InterruptedException e){
                    throw new RuntimeException(e);
                }
            }
        }
        return a;
    }
    public Action waitForSlides(int pos, boolean up, boolean action){
        Action a = new WaitForSlides(pos,up);
        if (!action){
            while (opMode&&!a.run()){
                try{
                    Thread.sleep(80);
                }catch(InterruptedException e){
                    throw new RuntimeException(e);
                }
            }
        }
        return a;
    }
    public Action waitForSlides(boolean action){
        return waitForSlides(leftM.getTargetPosition(),100,action);
    }
    public Action zeroArm(boolean action){
        Action a = new ZeroArm();
        if (!action){
            while (opMode&&!a.run()){
                try{
                    Thread.sleep(80);
                }catch(InterruptedException e){
                    throw new RuntimeException(e);
                }
            }
        }
        return a;
    }
    public class ZeroArm implements Action{
        int timesAtZero = 0;

        boolean init=false;
        @Override
        public boolean run() {
            if (!init) {
                armZeroing = true;
                rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightArm.setPower(-1);
                init=true;
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
            }
            else {
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
    public Action zeroSlides(boolean action){
        Action a = new ZeroSlides();
        if (!action){
            while (opMode&&!a.run()){
                try{
                    Thread.sleep(80);
                }catch(InterruptedException e){
                    throw new RuntimeException(e);
                }
            }
        }
        return a;
    }
    public class ZeroSlides implements Action{
        int timesAtZero = 0;
        double power=-0.7;
        int vel=20;
        boolean init=false;

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
                init=true;
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
            }
            else{
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
