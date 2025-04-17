package pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "LeftAuto4")
public class LeftAuto4 extends OpMode {
    private Follower follower;
    private Telemetry telemetryA;
    private Robot robot;
    private Timer pathTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(8, 104, Math.toRadians(0));
    private final Pose score = new Pose(13, 121, Math.toRadians(-45));
    private final Pose scorePre = new Pose(14,119,Math.toRadians(-45));
    private final Pose gather1Pose = new Pose(28, 114, Math.toRadians(0));
    private final Pose gather2Pose = new Pose(27.5, 123, Math.toRadians(0));
    private final Pose gather3Pose = new Pose(44.5,118,Math.toRadians(90));
    private final Pose gather3Inter = new Pose(41,100,Math.toRadians(90));
    private final Pose parkPoseInter = new Pose(58,120,Math.toRadians(-90));
    private final Pose basePickupPose = new Pose(60,92,Math.toRadians(-90));
    private final Pose parkPose = new Pose(58, 89, Math.toRadians(-90));
    //private final Pose parkControlPose = new Pose(8, 50, Math.toRadians(0));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private PathChain scorePreload,gather1,score1,gather2,score2,gather3,score3,gather4,score4,gather5,score5,park;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths(double x, double y, double angle,double x2,double y2,double angle2) {
        //int armPickup =0;
        int armScore = 2000;
        int lowSlides=500;
        int highSlides=2500;
        int raiseSlidesArmValue=1100;
        int shoulderFlip=1700;
        int letGo=2350;
        int armDown=1950;


        double offsetx = -1*(x-Math.cos(Math.toRadians(angle)));
        double offsetSlides = y+3-Math.sin(Math.toRadians(angle));
        int slidesVal = 200+(int)(offsetSlides*78);
        Pose gather4Pose = new Pose(basePickupPose.getX()+offsetx,basePickupPose.getY(),basePickupPose.getHeading());
        double offsetx2 = -1*(x2-Math.cos(Math.toRadians(angle2)));
        double offsetSlides2 = y2+3-Math.sin(Math.toRadians(angle2));
        int slidesVal2 = 200+(int)(offsetSlides2*78);
        Pose gather5Pose = new Pose(basePickupPose.getX()+offsetx2,basePickupPose.getY(),basePickupPose.getHeading());
        Pose scoreAdjust = score;

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose),new Point(scorePre)))
                .setLinearHeadingInterpolation(startPose.getHeading(),scorePre.getHeading())
                .addParametricCallback(0, () -> robot.moveShoulder(Robot.SHOULDERSCORE,false))
                .addParametricCallback(0,() -> robot.moveArm(armScore,false))
                //.addParametricCallback(0,()->robot.moveWrist(90*Robot.degToWrist))
                .addParametricCallback(0, () -> robot.actions.add(new SequentialAction(robot.waitForArm(raiseSlidesArmValue,true,true),robot.moveSlides(highSlides,true))))
                .addParametricCallback(0,() -> robot.actions.add(new SequentialAction(robot.waitForSlides(shoulderFlip-100,true,true),robot.moveShoulder(Robot.SHOULDERBASKET,true))))
                .addParametricCallback(0.95,() -> robot.actions.add(new SequentialAction(robot.waitForSlides(letGo+100,true,true),robot.moveClaw(Robot.CLAWOPEN,true))))
                .build();

        gather1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePre),new Point(gather1Pose)))
                .setLinearHeadingInterpolation(scorePre.getHeading(),gather1Pose.getHeading())
                //.setZeroPowerAccelerationMultiplier(2)
                .addParametricCallback(0, () -> robot.moveShoulder(Robot.SHOULDERSCORE,false))
                .addParametricCallback(0,()->robot.moveWrist(Robot.WRISTNORMAL,false))
                .addParametricCallback(0, () -> robot.moveArm(200,false))
                .addParametricCallback(0, () -> robot.actions.add(new SequentialAction(robot.waitForArm(armDown,false,true),robot.moveSlides(lowSlides-78,true))))
                .build();

        score1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(gather1Pose),new Point(score)))
                .setLinearHeadingInterpolation(gather1Pose.getHeading(),score.getHeading())
                //.setZeroPowerAccelerationMultiplier(1)
                .addParametricCallback(0,() -> robot.moveArm(armScore,false))
                //.addParametricCallback(0,()->robot.moveWrist(90*Robot.degToWrist))
                .addParametricCallback(0, () -> robot.actions.add(new SequentialAction(robot.moveShoulder(Robot.SHOULDERMID,true),robot.moveSlides(0,true),robot.waitForSlides(0,50,true),robot.zeroSlides(true),robot.waitForArm(raiseSlidesArmValue,true,true),robot.moveSlides(highSlides,true))))
                .addParametricCallback(0,() -> robot.actions.add(new SequentialAction(robot.waitForSlides(shoulderFlip,true,true),robot.moveShoulder(Robot.SHOULDERBASKET,true))))
                .addParametricCallback(0.95,() -> robot.actions.add(new SequentialAction(robot.waitForSlides(letGo,true,true),robot.moveClaw(Robot.CLAWOPEN,true))))
                .build();
        gather2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score),new Point(gather2Pose)))
                .setLinearHeadingInterpolation(score.getHeading(),gather2Pose.getHeading())
                //.setZeroPowerAccelerationMultiplier(2)
                .addParametricCallback(0, () -> robot.moveShoulder(Robot.SHOULDERSCORE,false))
                .addParametricCallback(0,()->robot.moveWrist(Robot.WRISTNORMAL,false))
                .addParametricCallback(0, () -> robot.moveArm(200,false))
                .addParametricCallback(0, () -> robot.actions.add(new SequentialAction(robot.waitForArm(armDown,false,true),robot.moveSlides(lowSlides,true))))
                .build();
        //scoreAdjust.setX(scoreAdjust.getX()-2.5);
        score2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(gather2Pose),new Point(scoreAdjust)))
                .setLinearHeadingInterpolation(gather2Pose.getHeading(),score.getHeading())
                .addParametricCallback(0,() -> robot.moveArm(armScore,false))
                //.addParametricCallback(0,()->robot.moveWrist(90*Robot.degToWrist))
                .addParametricCallback(0, () -> robot.actions.add(new SequentialAction(robot.moveShoulder(Robot.SHOULDERMID,true),robot.moveSlides(0,true),robot.waitForSlides(0,50,true),robot.zeroSlides(true),robot.waitForArm(raiseSlidesArmValue,true,true),robot.moveSlides(highSlides,true))))
                .addParametricCallback(0,() -> robot.actions.add(new SequentialAction(robot.waitForSlides(shoulderFlip,true,true),robot.moveShoulder(Robot.SHOULDERBASKET,true))))
                .addParametricCallback(0.95,() -> robot.actions.add(new SequentialAction(robot.waitForSlides(letGo,true,true),robot.moveClaw(Robot.CLAWOPEN,true))))
                .build();
        gather3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scoreAdjust),new Point(gather3Inter),new Point(gather3Pose)))
                .setLinearHeadingInterpolation(score.getHeading(),gather3Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(3)
                .addParametricCallback(0, () -> robot.moveShoulder(Robot.SHOULDERSCORE,false))
                .addParametricCallback(0,() -> robot.moveWrist(Robot.degToWrist *270,false))
                .addParametricCallback(0, () -> robot.moveArm(200,false))
                .addParametricCallback(0, () -> robot.actions.add(new SequentialAction(robot.waitForArm(armDown,false,true),robot.moveSlides(375,true))))
                .build();
        //scoreAdjust.setX(scoreAdjust.getX()+1.5);
        score3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(gather3Pose),new Point(scoreAdjust)))
                .setLinearHeadingInterpolation(gather3Pose.getHeading(),score.getHeading())
                .setZeroPowerAccelerationMultiplier(2.5)
                .addParametricCallback(0,() -> robot.moveWrist(Robot.WRISTNORMAL,false))
                .addParametricCallback(0,() -> robot.moveArm(armScore,false))
                //.addParametricCallback(0,()->robot.moveWrist(90*Robot.degToWrist))
                .addParametricCallback(0, () -> robot.actions.add(new SequentialAction(robot.moveShoulder(Robot.SHOULDERMID,true),robot.moveSlides(0,true),robot.waitForSlides(0,50,true),robot.zeroSlides(true),robot.waitForArm(raiseSlidesArmValue,true,true),robot.moveSlides(highSlides,true))))
                .addParametricCallback(0,() -> robot.actions.add(new SequentialAction(robot.waitForSlides(shoulderFlip,true,true),robot.moveShoulder(Robot.SHOULDERBASKET,true))))
                .addParametricCallback(0.95,() -> robot.actions.add(new SequentialAction(robot.waitForSlides(letGo,true,true),robot.moveClaw(Robot.CLAWOPEN,true))))
                .build();
        gather4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(score),new Point(parkPoseInter),new Point(gather4Pose)))
                .setLinearHeadingInterpolation(score.getHeading(),gather4Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(4)
                .addParametricCallback(0, () -> robot.moveShoulder(Robot.SHOULDERSCORE,false))
                .addParametricCallback(0,() -> robot.moveWrist(Robot.WRISTNORMAL,false))
                .addParametricCallback(0.2, () -> robot.moveWrist(Robot.degToWrist * (90 - angle) + Robot.WRISTNORMAL,false))
                .addParametricCallback(0, () -> robot.actions.add(robot.moveArm(250,true)))
                .addParametricCallback(0, () -> robot.actions.add(new SequentialAction(robot.waitForArm(armDown,false,true),robot.moveSlides(lowSlides,true))))
                .addParametricCallback(0.3, () -> robot.moveShoulder(Robot.SHOULDERMID,false))
                .addParametricCallback(0.9, () -> robot.moveSlides(slidesVal,false))
                .addParametricCallback(0.95, () -> robot.moveShoulder(Robot.SHOULDERSCORE,false))
                //.addParametricCallback(1, () -> robot.runAfterDelay(() -> robot.zeroArm(),0))
                .addPath(new BezierLine(new Point(gather4Pose),new Point(new Pose(gather4Pose.getX(),gather4Pose.getY()-2,gather4Pose.getHeading()))))
                .setConstantHeadingInterpolation(gather4Pose.getHeading())
                .build();
        //scoreAdjust.setX(scoreAdjust.getX()-1);
        score4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(gather4Pose),new Point(parkPoseInter),new Point(scoreAdjust)))
                .setLinearHeadingInterpolation(gather4Pose.getHeading(),score.getHeading())
                .setZeroPowerAccelerationMultiplier(4)
                .addParametricCallback(0,() -> robot.moveWrist(Robot.WRISTNORMAL,false))
                .addParametricCallback(0,() -> robot.moveSlides(300,false))
                .addParametricCallback(0,() -> robot.moveArm(armScore,false))
                //.addParametricCallback(0,()->robot.moveWrist(90*Robot.degToWrist))
                .addParametricCallback(0, () -> robot.actions.add(new SequentialAction(robot.moveShoulder(Robot.SHOULDERMID,true),robot.moveSlides(0,true),robot.waitForSlides(0,50,true),robot.zeroSlides(true),robot.waitForArm(raiseSlidesArmValue,true,true),robot.moveSlides(highSlides,true))))
                .addParametricCallback(0.95,() -> robot.actions.add(new SequentialAction(robot.waitForSlides(shoulderFlip,true,true),robot.moveShoulder(Robot.SHOULDERBASKET,true))))
                .addParametricCallback(0.99,() -> robot.actions.add(new SequentialAction(robot.waitForSlides(letGo,true,true),robot.moveClaw(Robot.CLAWOPEN,true))))
                .build();
        gather5 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(score),new Point(parkPoseInter),new Point(gather5Pose)))
                .setLinearHeadingInterpolation(score.getHeading(),gather5Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(4)
                .addParametricCallback(0, () -> robot.moveShoulder(Robot.SHOULDERSCORE,false))
                .addParametricCallback(0,() -> robot.moveWrist(Robot.WRISTNORMAL,false))
                .addParametricCallback(0.2, () -> robot.moveWrist(Robot.degToWrist * (90 - angle2) + Robot.WRISTNORMAL,false))
                .addParametricCallback(0, () -> robot.actions.add(robot.moveArm(250,true)))
                .addParametricCallback(0, () -> robot.actions.add(new SequentialAction(robot.waitForArm(armDown,false,true),robot.moveSlides(lowSlides,true))))
                .addParametricCallback(0.3, () -> robot.moveShoulder(Robot.SHOULDERMID,false))
                .addParametricCallback(0.9, () -> robot.moveSlides(slidesVal2,false))
                .addParametricCallback(0.95, () -> robot.moveShoulder(Robot.SHOULDERSCORE,false))
                //.addParametricCallback(1, () -> robot.runAfterDelay(() -> robot.zeroArm(),0))
                .addPath(new BezierLine(new Point(gather5Pose),new Point(new Pose(gather4Pose.getX(),gather5Pose.getY()-1,gather5Pose.getHeading()))))
                .setConstantHeadingInterpolation(gather5Pose.getHeading())
                .build();
        //scoreAdjust.setX(scoreAdjust.getX()-1);
        score5 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(gather5Pose),new Point(parkPoseInter),new Point(scoreAdjust)))
                .setLinearHeadingInterpolation(gather4Pose.getHeading(),score.getHeading())
                .setZeroPowerAccelerationMultiplier(4)
                .addParametricCallback(0,() -> robot.moveWrist(Robot.WRISTNORMAL,false))
                .addParametricCallback(0,() -> robot.moveSlides(300,false))
                .addParametricCallback(0,() -> robot.moveArm(armScore,false))
                //.addParametricCallback(0,()->robot.moveWrist(90*Robot.degToWrist))
                .addParametricCallback(0, () -> robot.actions.add(new SequentialAction(robot.moveShoulder(Robot.SHOULDERMID,true),robot.moveSlides(0,true),robot.waitForSlides(0,50,true),robot.zeroSlides(true),robot.waitForArm(raiseSlidesArmValue,true,true),robot.moveSlides(highSlides,true))))
                .addParametricCallback(0.95,() -> robot.actions.add(new SequentialAction(robot.waitForSlides(shoulderFlip,true,true),robot.moveShoulder(Robot.SHOULDERBASKET,true))))
                .addParametricCallback(0.99,() -> robot.actions.add(new SequentialAction(robot.waitForSlides(letGo,true,true),robot.moveClaw(Robot.CLAWOPEN,true))))
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scoreAdjust),new Point(parkPoseInter),new Point(parkPose)))
                .setLinearHeadingInterpolation(score.getHeading(),parkPoseInter.getHeading())
                .addParametricCallback(0, () -> robot.moveShoulder(Robot.SHOULDERSCORE,false))
                .addParametricCallback(0,()->robot.moveWrist(Robot.WRISTNORMAL,false))
                .addParametricCallback(0, () -> robot.moveArm(1100,false))
                .addParametricCallback(0.1, () -> robot.moveSlides(800,false))
                .addParametricCallback(0.5,() -> robot.moveShoulder(Robot.SHOULDERBASKET,false))

                .setLinearHeadingInterpolation(parkPoseInter.getHeading(),parkPose.getHeading())
                .setZeroPowerAccelerationMultiplier(4)
                .addParametricCallback(0.8, () -> robot.moveShoulder(Robot.SHOULDERSCORE,false))
                .addParametricCallback(.98, this::requestOpModeStop)
                .build();
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload,true);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    try{
                        Thread.sleep(300);
                    }catch (InterruptedException e){
                        throw new RuntimeException(e);
                    }
                    follower.followPath(gather1,true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    robot.waitForSlides(false);
                    robot.actions.add(robot.zeroArm(true));
                    robot.waitForArm(150,false,false);
                    robot.moveClaw(Robot.CLAWCLOSE,false);
                    try{
                        Thread.sleep(300);
                    }catch(InterruptedException e){
                        throw new RuntimeException(e);
                    }
                    robot.moveSlides(300,false);
                    follower.followPath(score1,true);
                    setPathState(3);
                }
                break;

            case 3:
                if(!follower.isBusy()) {
                    robot.waitForArm(false);
                    robot.waitForSlides(false);
                    follower.followPath(gather2,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    robot.waitForSlides(false);
                    robot.actions.add(robot.zeroArm(true));
                    robot.waitForArm(150,false,false);
                    robot.moveClaw(Robot.CLAWCLOSE,false);
                    try{
                        Thread.sleep(300);
                    }catch(InterruptedException e){
                        throw new RuntimeException(e);
                    }
                    robot.moveSlides(300,false);
                    follower.followPath(score2,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    robot.waitForArm(false);
                    robot.waitForSlides(false);
                    follower.followPath(gather3,true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    robot.waitForSlides(false);
                    robot.actions.add(robot.zeroArm(true));
                    robot.waitForArm(150,false,false);
                    robot.moveClaw(Robot.CLAWCLOSE,false);
                    try{
                        Thread.sleep(300);
                    }catch(InterruptedException e){
                        throw new RuntimeException(e);
                    }
                    robot.moveSlides(300,false);
                    follower.followPath(score3,true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    robot.waitForArm(false);
                    robot.waitForSlides(false);
                    Constants.setConstants(FConstants.class, LConstants.class);
                    buildPaths(offX,offY,offAngle,offX2,offY2,offAngle2);
                    follower.followPath(gather4);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()&&follower.getVelocity().getMagnitude()<4) {
                    setPathState(9);
                }
            case 9:
                if (!follower.isBusy()){
                    robot.actions.add(robot.zeroArm(true));
                    robot.waitForArm(100,false,false);
                    robot.moveClaw(Robot.CLAWCLOSE,false);
                    try{
                        Thread.sleep(150);
                    }catch(InterruptedException e){throw new RuntimeException(e);}
                    robot.actions.add(new SequentialAction(
                            new SleepAction(100),
                        robot.moveShoulder(Robot.SHOULDERMID,true),
                        robot.moveWrist(Robot.WRISTNORMAL,true)
                    ));
                    follower.followPath(score4);
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    robot.waitForArm(false);
                    robot.waitForSlides(false);
                    follower.followPath(gather5,true);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()&&follower.getVelocity().getMagnitude()<4) {
                    setPathState(12);
                }
            case 12:
                if (!follower.isBusy()){
                    robot.actions.add(robot.zeroArm(true));
                    robot.waitForArm(100,false,false);
                    robot.moveClaw(Robot.CLAWCLOSE,false);
                    try{
                        Thread.sleep(150);
                    }catch(InterruptedException e){throw new RuntimeException(e);}
                    robot.actions.add(new SequentialAction(
                            new SleepAction(100),
                        robot.moveShoulder(Robot.SHOULDERMID,true),
                        robot.moveWrist(Robot.WRISTNORMAL,true)
                    ));
                    follower.followPath(score5);
                    setPathState(13);
                }
                break;
            case 13:
                if(!follower.isBusy()) {
                    robot.waitForArm(false);
                    robot.waitForSlides(false);
                    follower.followPath(park,true);
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy()){
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        robot.executeActionSequence();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetryA.addData("path state", pathState);
        telemetryA.addData("armPos",robot.rightArm.getCurrentPosition());
        telemetryA.addData("LeftM position",robot.leftM.getCurrentPosition());
        telemetryA.addData("LeftM2 position",robot.leftM2.getCurrentPosition());
        telemetryA.addData("RightM position",robot.rightM.getCurrentPosition());
        follower.telemetryDebug(telemetryA);
        //telemetryA.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        robot=new Robot(hardwareMap,true);
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        Constants.setConstants(FConstants.class, LConstants.class);
        FollowerConstants.zeroPowerAccelerationMultiplier=1.75;
        FollowerConstants.lateralZeroPowerAcceleration=-45;
        FollowerConstants.forwardZeroPowerAcceleration=-18;
        FollowerConstants.xMovement=40;
        FollowerConstants.yMovement=30;
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        //robot.initCamera();
        //robot.disableProcessor();
        buildPaths(0,0,90,0,0,90);
        robot.moveSweep(Robot.SWEEPUP,false);
        robot.moveUp(Robot.UPDOWN,false);
        robot.moveHang(Robot.HANGCLOSE,false);
        robot.moveShoulder(Robot.SHOULDERPICKUP,false);
        robot.moveClaw(Robot.CLAWCLOSE,false);
        robot.moveWrist(Robot.WRISTNORMAL,false);
        robot.disengageActuator(false);
        robot.moveArm(1000,false);
        //robot.disableProcessor();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    double offX=0;
    double offY=0;
    double offAngle=90;
    double offX2=0;
    double offY2=0;
    double offAngle2=90;
    boolean built=false;
    Gamepad currentGamepad = new Gamepad();
    Gamepad previousGamepad = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();
    @Override
    public void init_loop() {
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad1);
        if (currentGamepad.dpad_left&&!previousGamepad.dpad_left){
            built=false;
            offX-=1;
        }
        else if (currentGamepad.dpad_right&&!previousGamepad.dpad_right) {
            built=false;
            offX += 1;
        }
        else if (currentGamepad.dpad_down&&!previousGamepad.dpad_down){
            built=false;
            offY-=1;
        }
        else if (currentGamepad.dpad_up&&!previousGamepad.dpad_up) {
            built=false;
            offY += 1;
        }
        else if (currentGamepad.right_bumper&&!previousGamepad.right_bumper){
            built=false;
            offAngle-=10;
        }
        else if (currentGamepad.left_bumper&&!previousGamepad.left_bumper) {
            built=false;
            offAngle += 10;
        }
        else if (currentGamepad.a&&!previousGamepad.a){
            built=true;
            buildPaths(offX,offY,offAngle,offX2,offY2,offAngle2);
        }
        telemetry.addData("built?",built);
        telemetry.addData("offsetX",offX);
        telemetry.addData("offsetY",offY);
        telemetry.addData("offsetAngle",offAngle);

        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);
        if (currentGamepad2.dpad_left&&!previousGamepad2.dpad_left){
            built=false;
            offX2-=1;
        }
        else if (currentGamepad2.dpad_right&&!previousGamepad2.dpad_right) {
            built=false;
            offX2 += 1;
        }
        else if (currentGamepad2.dpad_down&&!previousGamepad2.dpad_down){
            built=false;
            offY2-=1;
        }
        else if (currentGamepad2.dpad_up&&!previousGamepad2.dpad_up) {
            built=false;
            offY2 += 1;
        }
        else if (currentGamepad2.right_bumper&&!previousGamepad2.right_bumper){
            built=false;
            offAngle2-=10;
        }
        else if (currentGamepad2.left_bumper&&!previousGamepad2.left_bumper) {
            built=false;
            offAngle2 += 10;
        }
        telemetry.addData("offsetX2",offX2);
        telemetry.addData("offsetY2",offY2);
        telemetry.addData("offsetAngle2",offAngle2);
        telemetry.addLine("Press A when finished putting in offsets");
        telemetry.update();
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}

