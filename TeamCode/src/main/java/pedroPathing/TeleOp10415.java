package pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
//import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name = "TeleOp10415")
public class TeleOp10415 extends OpMode {
    Pose initialPose = new Pose(15, 25, Math.toRadians(0));
    Follower follower;
        private PoseUpdater poseUpdater;
        private DashboardPoseTracker dashboardPoseTracker;

    public Robot robot;
    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad currentGamepad2 = new Gamepad();
    public Gamepad previousGamepad1 = new Gamepad();
    public Gamepad previousGamepad2 = new Gamepad();
    public boolean init = false;
    public int armDownAdjust = 0;
    public int slideUpAdjust = 0;
    public int armDownAdjustInc = 0; //25;
    public int slideUpAdjustInc = 0;
    public boolean pullingUp = false;
    public PathChain scoreLotsOfShit;
    //    public double scoreAdjust=0.8;
    private final Pose pickup = new Pose(10.5, 38, Math.toRadians(0));
    private final Pose startPose = new Pose(pickup.getX() - 2.5, 37, pickup.getHeading());
    private final Pose score = new Pose(35, 66, Math.toRadians(10));
    private final Pose scoreFirst = new Pose(score.getX(), score.getY() + 2, score.getHeading());
    private final Pose scoreHelp = new Pose(score.getX()-4, score.getY()-4);

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(initialPose);
                poseUpdater = new PoseUpdater(hardwareMap);
                dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);
        robot = new Robot(hardwareMap, false);

        int armPickup = 2000;
        int armScore = 1500;
        int lowSlides = 1150;
        int highSlides = 1350;

        //        double clawCloseAdjust=0.0022;
        double clawClose = 0.97; //+clawCloseAdjust*14;
        double liftShoulder = 0;
        int liftShoulderWait = 100;
        double lowerShoulder = 0.01;
        double clawOpen = 0;
        int clawOpenWait = 150;
        double slidesUp = 0.88;
        double waitSlides = 0.01;
        int downPos = 20;
        PathBuilder builder = follower.pathBuilder();
        builder.addPath(new BezierCurve(new Point(startPose), new Point(scoreHelp), new Point(scoreFirst)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scoreFirst.getHeading())
                //.setZeroPowerAccelerationMultiplier(8)
                .addParametricCallback(0, () -> robot.moveArm(armScore, false))
                .addParametricCallback(0, () -> robot.moveSlides(lowSlides, false))
                .addParametricCallback(liftShoulder, () -> robot.actions.add(new SequentialAction(new SleepAction(liftShoulderWait), robot.moveShoulder(Robot.SHOULDERSCORE, true))))
                .addParametricCallback(slidesUp, () -> robot.moveSlides(highSlides, false));
        //score.setY(score.getY()+scoreAdjust*8);
        builder
                .addPath(new BezierLine(new Point(scoreFirst), new Point(pickup)))
                .setLinearHeadingInterpolation(scoreFirst.getHeading(), pickup.getHeading())
                //.setZeroPowerAccelerationMultiplier(8)
                .addParametricCallback(clawOpen, () -> robot.actions.add(new SequentialAction(new SleepAction(clawOpenWait), robot.moveClaw(Robot.CLAWOPEN, true))))
                .addParametricCallback(0, () -> robot.moveArm(armPickup, false))
                .addParametricCallback(waitSlides, () -> robot.actions.add(new SequentialAction(robot.moveSlides(downPos, true), robot.waitForSlides(100, false, true), robot.zeroSlides(true))))
                .addParametricCallback(lowerShoulder, () -> robot.moveShoulder(Robot.SHOULDERPICKUP, false))
                .addParametricCallback(clawClose, () -> robot.moveClaw(Robot.CLAWCLOSE, false));
        for (int i = 0; i < 15; i++) {
            builder.addPath(new BezierCurve(new Point(pickup), new Point(scoreHelp),new Point(score)))
                    .setLinearHeadingInterpolation(pickup.getHeading(), score.getHeading())
                    //.setZeroPowerAccelerationMultiplier(8)
                    .addParametricCallback(0, () -> robot.moveArm(armScore, false))
                    .addParametricCallback(0.05, () -> robot.moveSlides(lowSlides, false))
                    .addParametricCallback(liftShoulder, () -> robot.actions.add(new SequentialAction(new SleepAction(liftShoulderWait), robot.moveShoulder(Robot.SHOULDERSCORE, true))))
                    .addParametricCallback(slidesUp, () -> robot.moveSlides(highSlides, false));
            pickup.setY(pickup.getY() + 0.3);
            pickup.setX(pickup.getX() - 0.2);
            builder
                    .addPath(new BezierLine(new Point(score), new Point(pickup)))
                    .setLinearHeadingInterpolation(score.getHeading(), pickup.getHeading())
                    //.setZeroPowerAccelerationMultiplier(8)
                    .addParametricCallback(clawOpen, () -> robot.actions.add(new SequentialAction(new SleepAction(clawOpenWait), robot.moveClaw(Robot.CLAWOPEN, true))))
                    .addParametricCallback(0, () -> robot.moveArm(armPickup, false))
                    .addParametricCallback(waitSlides, () -> robot.actions.add(new SequentialAction(robot.moveSlides(downPos, true), robot.waitForSlides(100, false, true), robot.zeroSlides(true))))
                    .addParametricCallback(lowerShoulder, () -> robot.moveShoulder(Robot.SHOULDERPICKUP, false))
                    .addParametricCallback(clawClose, () -> robot.moveClaw(Robot.CLAWCLOSE, false));
            score.setY(score.getY() - 0.5);
            score.setX(score.getX() - 0.2);
            //clawClose-=clawCloseAdjust;
        }
        scoreLotsOfShit = builder.build();
    }
    boolean following = false;
    boolean justEnded = false;
    @Override
    public void loop() {
        follower.update();
        robot.doLotsOfGoodThings();
        if (!init) {
            init = true;
            robot.moveShoulder(Robot.SHOULDERMID, false);
            robot.moveWrist(Robot.degToWrist * 180, false);
            robot.moveClaw(Robot.CLAWCLOSE, false);
            robot.moveSweep(Robot.SWEEPUP, false);
            robot.moveUp(Robot.UPDOWN, false);
            robot.moveHang(Robot.HANGMID, false);
            follower.startTeleopDrive();
            for (String i: "fl,bl,br,fr".split(",")) {
                hardwareMap.dcMotor.get(i).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            robot.rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        if (currentGamepad1.back && !previousGamepad1.back) {
            if (!follower.isBusy()) {
                following = true;
                follower.setPose(startPose);
                follower.followPath(scoreLotsOfShit);
            } else {
                following = false;
                justEnded = true;
                follower.breakFollowing();
                follower.startTeleopDrive();
                for (String i: "fl,bl,br,fr".split(",")) {
                    hardwareMap.dcMotor.get(i).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }
        }
        if (!following || !follower.isBusy()) {
            following = false;
            poseUpdater.update();
            dashboardPoseTracker.update();
            Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
            Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
            Drawing.sendPacket();
            if (justEnded) {
                justEnded = false;
                follower.breakFollowing();
                follower.startTeleopDrive();
                for (String i: "fl,bl,br,fr".split(",")) {
                    hardwareMap.dcMotor.get(i).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }
            robot.checkResetSlideEncoders();
            robot.checkResetArmEncoder();
            if (!pullingUp) {
                robot.moveSlidesNoEncoder((gamepad2.right_trigger - gamepad2.left_trigger));
            }
            robot.moveArmNoEncoder(-gamepad2.left_stick_y);

            if (gamepad1.right_bumper && gamepad1.left_bumper) {
                pullingUp = true;
                robot.engageActuator(false);
            } else {
                robot.disengageActuator(false);
            }

            if (pullingUp) {
                robot.moveSlidesNoEncoder(-1);
            }

            if (gamepad1.start) {
                pullingUp = false;
            }

            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                robot.moveShoulder(Robot.SHOULDERSCORE, false);
                robot.moveWrist(Robot.WRISTNORMAL, false);
                robot.moveArm(2200, false);
                robot.actions.add(new SequentialAction(
                        robot.waitForArm(1100, true, true),
                        robot.moveSlides(2000, true)
                ));
            } else if ((currentGamepad2.dpad_right && !previousGamepad2.dpad_right)) {
                robot.moveShoulder(Robot.SHOULDERPICKUP, false);
                robot.moveWrist(Robot.degToWrist * 180, false);
                robot.moveClaw(Robot.CLAWOPEN, false);
                robot.moveArm(1800, false);
                robot.moveSlides(-20, false);
                robot.actions.add(new SequentialAction(
                        robot.waitForSlides(100, false, true),
                        robot.waitForArm(1600, true, true),
                        robot.zeroSlides(true)
                ));
            } else if ((currentGamepad2.dpad_left && !previousGamepad2.dpad_left)) {
                robot.moveArm(1500 - armDownAdjust, false);
                robot.moveShoulder(Robot.SHOULDERSCORE, false);
                robot.moveWrist(Robot.degToWrist * 180, false);
                robot.moveSlides(1075 + slideUpAdjust, false);
                armDownAdjust += armDownAdjustInc;
                slideUpAdjust += slideUpAdjustInc;
            } else if ((currentGamepad2.dpad_up && !previousGamepad2.dpad_up)) {
                robot.moveArm(2000, false);
                robot.moveSlides(200, false);
                robot.actions.add(new SequentialAction(
                        robot.waitForArm(1600, true, true),
                        robot.moveSlides(2500, true)
                ));
                robot.actions.add(new SequentialAction(
                        robot.moveShoulder(Robot.SHOULDERMID, true),
                        new SleepAction(300),
                        robot.moveShoulder(Robot.SHOULDERSCORE, true),
                        robot.waitForSlides(2000,true,true),
                        robot.moveShoulder(Robot.SHOULDERBASKET,true)
                ));
                /*
                robot.runAfterDelay(() -> {
                    robot.waitForArm(150, true);
                    //robot.moveShoulder(Robot.SHOULDERSCORE);
                    robot.moveClaw(Robot.CLAWLOOSE);
                    robot.waitForArm(600, true);
                    robot.moveClaw(Robot.CLAWCLOSE);
                }, 0);

                 */
                robot.moveWrist(Robot.degToWrist * 180, false);
            } else if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
                robot.actions.add(new SequentialAction(
                        robot.zeroArm(true),
                        robot.moveArm(200, true)
                ));
                robot.moveShoulder(Robot.SHOULDERMID, false);
                robot.moveWrist(Robot.degToWrist * 180, false);
                robot.actions.add(new SequentialAction(
                        robot.waitForArm(1860, false, true),
                        robot.moveSlides(200, true)
                ));
            }

            if (currentGamepad2.y && !previousGamepad2.y) {
                if (robot.shoulder.getPosition() < 0.8) {
                    robot.moveShoulder(Robot.SHOULDERSCORE, false);
                } else {
                    robot.moveShoulder(Robot.SHOULDERBASKET, false);
                    robot.moveWrist(Robot.degToWrist * 180, false);
                }
            }

            if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
                robot.moveWrist(Robot.degToWrist * 90, false);
            } else if ((currentGamepad2.b && !previousGamepad2.b) || (currentGamepad2.right_bumper && !previousGamepad2.right_bumper)) {
                robot.moveWrist(Robot.degToWrist * 180, false);
            }

            if (currentGamepad2.x && !previousGamepad2.x) {
                if (robot.claw.getPosition() > 0.7) {
                    robot.moveClaw(Robot.CLAWCLOSE, false);
                } else {
                    robot.moveClaw(Robot.CLAWOPEN, false);
                }
                if (robot.rightArm.getTargetPosition() == 2000 && robot.leftM.getCurrentPosition() >= 1500) {
                    robot.actions.add(new SequentialAction(new SleepAction(100), robot.moveShoulder(Robot.SHOULDERSCORE, true)));
                    robot.actions.add(new SequentialAction(
                            new SleepAction(400),
                            robot.moveShoulder(Robot.SHOULDERMID, true),
                            robot.moveWrist(Robot.degToWrist * 180, true),
                            robot.moveArm(200, true)
                    ));
                    robot.actions.add(new SequentialAction(
                            robot.waitForArm(1860, false, true),
                            robot.moveSlides(200, true)
                    ));
                }
            }

            if (currentGamepad2.a && !previousGamepad2.a) {
                if (robot.shoulder.getPosition() < 0.8) {
                    robot.moveShoulder(Robot.SHOULDERSCORE, false);
                } else {
                    robot.moveShoulder(Robot.SHOULDERMID, false);
                    robot.moveWrist(Robot.degToWrist * 180, false);
                }
            }
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -0.7 * gamepad1.right_stick_x, true);
        }
        //follower.update();

        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.addData("RightArm",robot.rightArm.getCurrentPosition());
        telemetry.addData("Arm",robot.rightArm.getTargetPosition());
        telemetry.addData("Up",robot.leftM.getPower());
        telemetry.addData("LeftSlide",robot.leftM.getCurrentPosition());
        telemetry.addData("LeftSlide2",robot.leftM2.getCurrentPosition());
        telemetry.addData("RightSlide",robot.rightM.getCurrentPosition());
        telemetry.addData("SlideMode",robot.leftM.getMode());
        telemetry.addData("pid coefficients",robot.rightArm.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        telemetry.update();
    }
}