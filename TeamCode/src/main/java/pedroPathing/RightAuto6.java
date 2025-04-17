package pedroPathing;

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

@Autonomous(name = "RightAuto6")
public class RightAuto6 extends OpMode {
    private Follower follower;
    private Telemetry telemetryA;
    private Robot robot;
    private Timer pathTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(8, 67, Math.toRadians(0));
    private final Pose scoreFirstHelp = new Pose(36, 78, Math.toRadians(0));
    private final Pose dropOffGather = new Pose(20, 42, Math.toRadians(0));
    private final Pose dropOffGatherTemp = new Pose(dropOffGather.getX(), 74);
    //private final Pose push = new Pose(25, 43, Math.toRadians(0));
    private final Pose pushInter = new Pose(35, 40.5, Math.toRadians(-25));
    private final Pose dropOff = new Pose(26, 37, Math.toRadians(-135));
    //private final Pose push2 = new Pose(25, 36, Math.toRadians(0));
    private final Pose pushInter2 = new Pose(35.5, 32, Math.toRadians(-30));
    private final Pose dropOff2 = new Pose(30, 28, Math.toRadians(-120));
    //private final Pose push3 = new Pose(58, 20, Math.toRadians(0));
    private final Pose pushInter3 = new Pose(35, 21.5, Math.toRadians(-30));
    private final Pose dropOff3 = new Pose(24, 23, Math.toRadians(-100));
    private final Pose dropOff3Help = new Pose(28, 20);
    private final Pose pickup = new Pose(10.5, 39, Math.toRadians(0));
    private final Pose scoreHelp = new Pose(20, 53);
    private final Pose score = new Pose(37, 68, Math.toRadians(0));

    private final Pose pickUpFromGround = new Pose(14, 44, Math.toRadians(-90));
    private final Pose finish = new Pose(10, 125, Math.toRadians(-45));
    private final Pose finish2 = new Pose(11, finish.getY() - 10, Math.toRadians(-90));
    private final Pose parkPose = new Pose(15, 25, Math.toRadians(-90));
    //private final Pose parkControlPose = new Pose(8, 50, Math.toRadians(0));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private PathChain scorePreload, gather, park; //,scoreLast,park;//,scorePath1;//,scorePath2,scorePath3,scorePath4,scorePath5;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths(double x, double y, double angle) {
        int armPickup = 2000;
        int armScore = 1500;
        int lowSlides = 1150;
        int highSlides = 1250;

        double clawClose = 0.90;
        double liftShoulder = 0;
        int liftShoulderWait = 100;
        double lowerShoulder = 0.01;
        double clawOpen = 0;
        int clawOpenWait = 150;
        double slidesUp = 0.88;
        double waitSlides = 0.01;
        int downPos = -20;
        int closeOnFirstWait = 30;
        int putScoreArmBack = 550;

        double offsety = -1 * (x - Math.cos(Math.toRadians(angle)));
        double offsetSlides = 1 * (y + 3 - Math.sin(Math.toRadians(angle)));
        int slideVal = 150 + (int)(offsetSlides * Robot.slideTicksToInch);
        Pose scoreFirst = new Pose(scoreFirstHelp.getX() - 0.5, scoreFirstHelp.getY() + offsety, scoreFirstHelp.getHeading());
        Pose scoreFirst2 = new Pose(scoreFirst.getX() + 2, scoreFirst.getY() - 2, scoreFirst.getHeading());
        Pose dropOffGatherDrift = new Pose(dropOffGather.getX() + 2, dropOffGather.getY(), dropOffGather.getHeading());
        Pose scoreFirstSpec = new Pose(score.getX(), score.getY() + 1, score.getHeading());

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scoreFirst)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scoreFirstHelp.getHeading())
                .setZeroPowerAccelerationMultiplier(8)
                .addParametricCallback(0, () -> robot.moveUp(Robot.UPSCORE, false))
                //.addParametricCallback(0,() -> robot.moveArm(robot.getArmForSlidesAndHeight(slideVal,4)))
                .addParametricCallback(0, () -> robot.moveArm(200, false))
                .addParametricCallback(0, () -> robot.moveShoulder(Robot.SHOULDERSCORE, false))
                .addParametricCallback(0.2, () -> robot.moveSlides(slideVal, false))
                .addParametricCallback(0.1, () -> robot.moveWrist(Robot.degToWrist * (90 - angle) + Robot.WRISTNORMAL, false))

                //.addParametricCallback(0, () -> robot.moveShoulder(Robot.SHOULDERSCORE))
//                .addParametricCallback(0.9, () -> robot.actions.add(new SequentialAction(
//                        new SleepAction(putScoreArmBack),
//                        robot.moveHang(Robot.HANGOPEN, true),
//                        robot.moveUp(Robot.UPDOWN, true)
//                )))
                .addPath(new BezierLine(new Point(scoreFirst), new Point(scoreFirst2)))
                .setConstantHeadingInterpolation(scoreFirst.getHeading())
                .setZeroPowerAccelerationMultiplier(2.5)
                .build();
        gather = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scoreFirst), new Point(dropOffGatherTemp), new Point(dropOffGather)))
                .setLinearHeadingInterpolation(scoreFirst.getHeading(), dropOffGather.getHeading())
                .setZeroPowerAccelerationMultiplier(8)
                //.addParametricCallback(0,() -> robot.moveHang(robot.HANGOPEN))
                //.addParametricCallback(0,() -> robot.moveUp(robot.UPDOWN))
                .addParametricCallback(0, () -> robot.moveShoulder(Robot.SHOULDERMID, false))
                .addParametricCallback(0, () -> robot.actions.add(new SequentialAction(robot.moveSlides(downPos, true), robot.waitForSlides(100, false, true), robot.zeroSlides(true))))
                .addParametricCallback(0.2, () -> robot.moveArm(armPickup, false))
                //.addParametricCallback(0.5, () -> robot.moveSlides(800))
                .addParametricCallback(0.5, () -> robot.moveShoulder(Robot.SHOULDERBASKET, false))
                .addParametricCallback(0.5, () -> robot.moveWrist(Robot.degToWrist * 90, false))
                .addParametricCallback(0.9, () -> robot.moveClaw(Robot.CLAWOPEN, false))

                .addPath(new BezierLine(new Point(dropOffGatherDrift), new Point(pushInter)))
                .setLinearHeadingInterpolation(dropOffGatherDrift.getHeading(), pushInter.getHeading())
                .addParametricCallback(0, () -> robot.actions.add(new SequentialAction(robot.moveSlides(0, true), robot.zeroSlides(true))))
                //.addParametricCallback(0,() -> robot.moveArm(armScore,0.4))
                .addParametricCallback(0.5, () -> robot.moveWrist(Robot.WRISTNORMAL, false))
                .addParametricCallback(0.5, () -> robot.moveShoulder(Robot.SHOULDERPICKUP, false))
                .addParametricCallback(0.8, () -> robot.moveSweep(Robot.SWEEPDOWN, false))

                //.addPath(new BezierLine(new Point(push),new Point(pushInter)))
                //.setLinearHeadingInterpolation(push.getHeading(),pushInter.getHeading())
                //.setPathEndVelocityConstraint(10)

                .addPath(new BezierLine(new Point(pushInter), new Point(dropOff)))
                .setLinearHeadingInterpolation(pushInter.getHeading(), dropOff.getHeading())

                //.addPath(new BezierLine(new Point(dropOff),new Point(push2)))
                //.setLinearHeadingInterpolation(dropOff.getHeading(),push2.getHeading())

                .addPath(new BezierLine(new Point(dropOff), new Point(pushInter2)))
                .setLinearHeadingInterpolation(dropOff.getHeading(), pushInter2.getHeading())
                //.setPathEndVelocityConstraint(10)
                .addParametricCallback(0, () -> robot.moveSweep(0.5, false))
                .addParametricCallback(0.9, () -> robot.moveSweep(Robot.SWEEPDOWN, false))

                .addPath(new BezierLine(new Point(pushInter2), new Point(dropOff2)))
                .setLinearHeadingInterpolation(pushInter2.getHeading(), dropOff2.getHeading())

                .addPath(new BezierLine(new Point(dropOff2), new Point(pushInter3)))
                .setLinearHeadingInterpolation(dropOff2.getHeading(), pushInter3.getHeading())
                .addParametricCallback(0, () -> robot.moveSweep(0.5, false))
                .addParametricCallback(0.96, () -> robot.moveSweep(Robot.SWEEPDOWN, false))
                //.addParametricCallback(0, () -> robot.moveArm(armScore))

                //.addPath(new BezierLine(new Point(push3),new Point(pushInter3)))
                //.setLinearHeadingInterpolation(push3.getHeading(),pushInter3.getHeading())

                .addPath(new BezierCurve(new Point(pushInter3), new Point(dropOff3Help), new Point(dropOff3)))
                //.addPath(new BezierCurve(new Point(pushInter3),new Point(dropOff3Help),new Point(dropOff3)))
                .setLinearHeadingInterpolation(pushInter3.getHeading(), dropOff3.getHeading())
                .addParametricCallback(0, () -> robot.moveClaw(Robot.CLAWOPEN, false))
                //.addParametricCallback(0, () -> robot.moveShoulder(Robot.SHOULDERPICKUP))
                .addParametricCallback(0, () -> robot.moveSlides(downPos, false))

                .addPath(new BezierLine(new Point(dropOff3), new Point(new Pose(pickup.getX() - 1.5, pickup.getY() - 2, pickup.getHeading()))))
                .setLinearHeadingInterpolation(dropOff3.getHeading(), pickup.getHeading())
                .addParametricCallback(0.1, () -> robot.moveSweep(Robot.SWEEPUP, false))
                //.addParametricCallback(0,() -> robot.moveArm(armPickup))
                .addParametricCallback(clawClose, () -> robot.actions.add(new SequentialAction(new SleepAction(closeOnFirstWait), robot.moveClaw(Robot.CLAWCLOSE, true))))
                //.build();
                //scorePath1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickup), new Point(scoreHelp), new Point(scoreFirstSpec)))
                .setConstantHeadingInterpolation(score.getHeading())
                .setZeroPowerAccelerationMultiplier(8)
                .addParametricCallback(0, () -> robot.moveArm(armScore, false))
                .addParametricCallback(0.2, () -> robot.moveSlides(lowSlides, false))
                .addParametricCallback(liftShoulder, () -> robot.actions.add(new SequentialAction(new SleepAction(liftShoulderWait), robot.moveShoulder(Robot.SHOULDERSCORE, true))))
                .addParametricCallback(slidesUp, () -> robot.moveSlides(highSlides, false))

                .addPath(new BezierLine(new Point(scoreFirstSpec), new Point(pickup)))
                .setConstantHeadingInterpolation(pickup.getHeading())
                .setZeroPowerAccelerationMultiplier(8)
                .addParametricCallback(clawOpen, () -> robot.actions.add(new SequentialAction(new SleepAction(clawOpenWait), robot.moveClaw(Robot.CLAWOPEN, true))))
                .addParametricCallback(0, () -> robot.moveArm(armPickup, false))
                .addParametricCallback(waitSlides, () -> robot.actions.add(new SequentialAction(robot.moveSlides(downPos, true), robot.waitForSlides(100, false, true), robot.zeroSlides(true))))
                .addParametricCallback(lowerShoulder, () -> robot.moveShoulder(Robot.SHOULDERPICKUP, false))
                .addParametricCallback(clawClose, () -> robot.moveClaw(Robot.CLAWCLOSE, false))
                //.build();

                //scorePath2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickup), new Point(scoreHelp), new Point(score)))
                .setConstantHeadingInterpolation(score.getHeading())
                .setZeroPowerAccelerationMultiplier(8)
                .addParametricCallback(0, () -> robot.moveArm(armScore, false))
                .addParametricCallback(0.2, () -> robot.moveSlides(lowSlides, false))
                .addParametricCallback(liftShoulder, () -> robot.actions.add(new SequentialAction(new SleepAction(liftShoulderWait), robot.moveShoulder(Robot.SHOULDERSCORE, true))))
                .addParametricCallback(slidesUp, () -> robot.moveSlides(highSlides, false))
                //.addParametricCallback(0.95, () -> robot.moveClaw(robot.CLAWOPEN))

                .addPath(new BezierLine(new Point(score), new Point(pickup)))
                .setConstantHeadingInterpolation(pickup.getHeading())
                .setZeroPowerAccelerationMultiplier(8)
                .addParametricCallback(clawOpen, () -> robot.actions.add(new SequentialAction(new SleepAction(clawOpenWait), robot.moveClaw(Robot.CLAWOPEN, true))))
                .addParametricCallback(0, () -> robot.moveArm(armPickup, false))
                .addParametricCallback(waitSlides, () -> robot.actions.add(new SequentialAction(robot.moveSlides(downPos, true), robot.waitForSlides(100, false, true), robot.zeroSlides(true))))
                .addParametricCallback(lowerShoulder, () -> robot.moveShoulder(Robot.SHOULDERPICKUP, false))
                .addParametricCallback(clawClose, () -> robot.moveClaw(Robot.CLAWCLOSE, false))
                //.build();

                //scorePath3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickup), new Point(scoreHelp), new Point(score)))
                .setConstantHeadingInterpolation(score.getHeading())
                .setZeroPowerAccelerationMultiplier(8)
                .addParametricCallback(0, () -> robot.moveArm(armScore, false))
                .addParametricCallback(0.2, () -> robot.moveSlides(lowSlides, false))
                .addParametricCallback(liftShoulder, () -> robot.actions.add(new SequentialAction(new SleepAction(liftShoulderWait), robot.moveShoulder(Robot.SHOULDERSCORE, true))))
                .addParametricCallback(slidesUp, () -> robot.moveSlides(highSlides, false))
                //.addParametricCallback(0.95, () -> robot.moveClaw(robot.CLAWOPEN))

                .addPath(new BezierLine(new Point(score), new Point(pickup)))
                .setConstantHeadingInterpolation(pickup.getHeading())
                .setZeroPowerAccelerationMultiplier(8)
                .addParametricCallback(clawOpen, () -> robot.actions.add(new SequentialAction(new SleepAction(clawOpenWait), robot.moveClaw(Robot.CLAWOPEN, true))))
                .addParametricCallback(0, () -> robot.moveArm(armPickup, false))
                .addParametricCallback(waitSlides, () -> robot.actions.add(new SequentialAction(robot.moveSlides(downPos, true), robot.waitForSlides(100, false, true), robot.zeroSlides(true))))
                .addParametricCallback(lowerShoulder, () -> robot.moveShoulder(Robot.SHOULDERPICKUP, false))
                .addParametricCallback(clawClose, () -> robot.moveClaw(Robot.CLAWCLOSE, false))
                // .build();

                //scorePath4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickup), new Point(scoreHelp), new Point(score)))
                .setConstantHeadingInterpolation(score.getHeading())
                .setZeroPowerAccelerationMultiplier(8)
                .addParametricCallback(0, () -> robot.moveArm(armScore, false))
                .addParametricCallback(0.2, () -> robot.moveSlides(lowSlides, false))
                .addParametricCallback(liftShoulder, () -> robot.actions.add(new SequentialAction(new SleepAction(liftShoulderWait), robot.moveShoulder(Robot.SHOULDERSCORE, true))))
                .addParametricCallback(slidesUp, () -> robot.moveSlides(highSlides + 50, false))
                //.addParametricCallback(0.95, () -> robot.moveClaw(robot.CLAWOPEN))

                .addPath(new BezierLine(new Point(score), new Point(pickup)))
                .setConstantHeadingInterpolation(pickup.getHeading())
                .setZeroPowerAccelerationMultiplier(8)
                .addParametricCallback(clawOpen, () -> robot.actions.add(new SequentialAction(new SleepAction(clawOpenWait), robot.moveClaw(Robot.CLAWOPEN, true))))
                .addParametricCallback(0, () -> robot.moveArm(armPickup, false))
                .addParametricCallback(waitSlides, () -> robot.actions.add(new SequentialAction(robot.moveSlides(downPos, true), robot.waitForSlides(100, false, true), robot.zeroSlides(true))))
                .addParametricCallback(lowerShoulder, () -> robot.moveShoulder(Robot.SHOULDERPICKUP, false))
                .addParametricCallback(clawClose, () -> robot.moveClaw(Robot.CLAWCLOSE, false))
                //        .build();

                // scorePath5 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickup), new Point(scoreHelp), new Point(score)))
                .setConstantHeadingInterpolation(score.getHeading())
                .setZeroPowerAccelerationMultiplier(8)
                .addParametricCallback(0, () -> robot.moveArm(armScore, false))
                .addParametricCallback(0.2, () -> robot.moveSlides(lowSlides, false))
                .addParametricCallback(liftShoulder, () -> robot.actions.add(new SequentialAction(new SleepAction(liftShoulderWait), robot.moveShoulder(Robot.SHOULDERSCORE, true))))
                .addParametricCallback(slidesUp, () -> robot.moveSlides(highSlides + 50, false))
                //.addParametricCallback(0.95, () -> robot.moveClaw(robot.CLAWOPEN))
                /*
                                .addPath(new BezierLine(new Point(score),new Point(parkPose)))
                                .setLinearHeadingInterpolation(score.getHeading(),parkPose.getHeading())
                                .addParametricCallback(clawOpen, () -> robot.runAfterDelay(() -> robot.moveClaw(Robot.CLAWOPEN),clawOpenWait))
                                .addParametricCallback(0.1, () ->robot.runAfterDelay(() -> robot.zeroArm(),0))
                                .addParametricCallback(waitSlides, () -> robot.moveSlides(0))
                                .addParametricCallback(lowerShoulder, () -> robot.moveShoulder(Robot.SHOULDERPICKUP))

                 */

                .addPath(new BezierLine(new Point(score), new Point(pickUpFromGround)))
                .setLinearHeadingInterpolation(score.getHeading(), pickUpFromGround.getHeading())
                .addParametricCallback(clawOpen, () -> robot.actions.add(new SequentialAction(new SleepAction(clawOpenWait), robot.moveClaw(Robot.CLAWOPEN, true))))
                .addParametricCallback(0.05, () -> robot.actions.add(robot.zeroArm(true)))
                .addParametricCallback(0.2, () -> robot.moveSlides(300, false))
                .addParametricCallback(lowerShoulder, () -> robot.moveShoulder(Robot.SHOULDERPICKUP, false))
                .addParametricCallback(0.5, () -> robot.moveShoulder(Robot.SHOULDERSCORE, false))
                //         .build();

                //scoreLast = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickUpFromGround), new Point(finish2)))
                .setLinearHeadingInterpolation(pickUpFromGround.getHeading(), finish2.getHeading())
                .setZeroPowerAccelerationMultiplier(8)
                .addParametricCallback(0, () -> robot.moveClaw(Robot.CLAWCLOSE, false))
                .addParametricCallback(0, () -> robot.moveSlides(150, false))
                .addParametricCallback(0, () -> robot.moveArm(2100, false))
                .addParametricCallback(0, () -> robot.actions.add(new SequentialAction(robot.waitForArm(900, true, true), robot.moveSlides(2500, true))))
                .addParametricCallback(0, () -> robot.actions.add(new SequentialAction(robot.waitForSlides(1800, true, true), robot.moveShoulder(Robot.SHOULDERBASKET, true), new SleepAction(200), robot.waitForSlides(2350, true, true), robot.moveClaw(Robot.CLAWOPEN, true))))
                //.addParametricCallback(0.95,() -> robot.runAfterDelay(() -> {},0))
                .addPath(new BezierLine(new Point(finish2), new Point(finish)))
                .setLinearHeadingInterpolation(finish2.getHeading(), finish.getHeading())
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(finish), new Point(parkPose)))
                .setConstantHeadingInterpolation(parkPose.getHeading())
                .setZeroPowerAccelerationMultiplier(8)
                .addParametricCallback(0, () -> robot.actions.add(robot.zeroArm(true)))
                .addParametricCallback(0, () -> robot.actions.add(new SequentialAction(robot.waitForArm(1900, false, true), robot.moveSlides(50, true))))
                .addParametricCallback(0, () -> robot.moveShoulder(Robot.SHOULDERSCORE, false))
                .addParametricCallback(0.2, () -> robot.moveShoulder(Robot.SHOULDERPICKUP, false))

                .addParametricCallback(1, this::requestOpModeStop)
                .build();
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() && follower.getVelocity().getMagnitude() < 4) {
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    robot.moveHang(Robot.HANGOPEN, false);
                    robot.moveUp(Robot.UPDOWN, false);
                    robot.zeroArm(false);
                    robot.moveClaw(Robot.CLAWCLOSE, false);
                    try {
                        Thread.sleep(200);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    robot.moveShoulder(Robot.SHOULDERMID, false);
                    robot.moveWrist(Robot.WRISTNORMAL, false);
                    try {
                        Thread.sleep(150);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    //robot.waitForArm(0,50,false);
                    follower.followPath(gather, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    robot.waitForSlides(false);
                    robot.waitForArm(false);
                    follower.followPath(park, true);
                    setPathState(4);
                }

            case 7:
                if (!follower.isBusy()) {
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
        robot.doLotsOfGoodThings();
        autonomousPathUpdate();
        //robot.doCameraThings(telemetryA);
        /*
                // Feedback to Driver Hub
                telemetryA.addData("path state", pathState);
                //telemetryA.addData("x", follower.getPose().getX());
                //telemetryA.addData("y", follower.getPose().getY());
                //telemetryA.addData("heading", follower.getPose().getHeading());
                telemetryA.addData("armPos",robot.rightArm.getCurrentPosition());
                telemetryA.addData("LeftM position",robot.leftM.getCurrentPosition());
                telemetryA.addData("LeftM2 position",robot.leftM2.getCurrentPosition());
                telemetryA.addData("RightM position",robot.rightM.getCurrentPosition());
                //follower.telemetryDebug(telemetryA);
                //telemetryA.update();

         */
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        robot = new Robot(hardwareMap, true);
        //telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        Constants.setConstants(FConstants.class, LConstants.class);
        FollowerConstants.lateralZeroPowerAcceleration = -60;
        FollowerConstants.forwardZeroPowerAcceleration = -25;
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths(0, 0, 90);
        robot.moveSweep(Robot.SWEEPUP, false);
        robot.moveUp(Robot.UPDOWN, false);
        robot.moveHang(Robot.HANGCLOSE, false);
        robot.moveShoulder(Robot.SHOULDERPICKUP, false);
        robot.moveClaw(Robot.CLAWOPEN, false);
        robot.moveWrist(Robot.WRISTNORMAL, false);
        robot.disengageActuator(false);
        robot.moveArm(1000, false);
    }

    /** This method is called continuously after Init while waiting for "play". **/
    double offX = 0;
    double offY = 0;
    double offAngle = 90;
    boolean built = false;
    Gamepad currentGamepad = new Gamepad();
    Gamepad previousGamepad = new Gamepad();
    @Override
    public void init_loop() {
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad1);
        if (currentGamepad.dpad_left && !previousGamepad.dpad_left) {
            built = false;
            offX -= 1;
        } else if (currentGamepad.dpad_right && !previousGamepad.dpad_right) {
            built = false;
            offX += 1;
        } else if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {
            built = false;
            offY -= 1;
        } else if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
            built = false;
            offY += 1;
        } else if (currentGamepad.right_bumper && !previousGamepad.right_bumper) {
            built = false;
            offAngle -= 10;
        } else if (currentGamepad.left_bumper && !previousGamepad.left_bumper) {
            built = false;
            offAngle += 10;
        } else if (currentGamepad.a && !previousGamepad.a) {
            built = true;
            buildPaths(offX, offY, offAngle);
        }
        telemetry.addData("built?", built);
        telemetry.addData("offsetX", offX);
        telemetry.addData("offsetY", offY);
        telemetry.addData("offsetAngle", offAngle);
        //double offsety = -offX + Math.cos(Math.toRadians(offAngle));
        //double offsetSlides = offY + 1 - Math.sin(Math.toRadians(offAngle));
        //telemetry.addData("effective offsety",offsety);
        //telemetry.addData("effective offsetSlides",offsetSlides);
        //telemetry.addData("slides running to",280 + (int) ((offsetSlides + (36-scoreFirstHelp.getX())) * 78));
        telemetry.addLine("Press A when finished putting in offsets");
        //telemetry.addLine("leftM: "+robot.leftM.getCurrentPosition()+", leftM2: "+robot.leftM2.getCurrentPosition()+", rightM: "+robot.rightM.getCurrentPosition());
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
        robot.stop();
    }
}