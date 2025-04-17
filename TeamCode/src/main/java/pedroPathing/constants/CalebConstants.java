package pedroPathing.constants;

public final class CalebConstants {
    public static final int SLEEPTIME = 20;

    public static final double FEEDFORWARD_ARM = (double) 1 /10440;

    public static final double SWEEPDOWN = 0;
    public static final double SWEEPUP = 1;
    public static final double UPSCORE = .95;
    public static final double UPDOWN = 0.14;
    public static final double HANGCLOSE = 0.3;
    public static final double HANGOPEN = 0.8;
    public static final double HANGMID = 0.4;
    public static final double CLAWOPEN = 0.825;
    public static final double CLAWCLOSE = 0.27;
    //    public static final double CLAWLOOSE = CLAWCLOSE + 0.07;
    public static final double WRISTNORMAL = 0.55;
    public static final double degToWrist = WRISTNORMAL / 180;
    public static final double SHOULDERSCORE = 0.94;
    public static final double SHOULDERPICKUP = 0.04;
    public static final double SHOULDERBASKET = 0.22;
    public static final double SHOULDERMID = 0.65;
    public static final double ACTUATORENGAGE = 0.7;
    public static final double ACTUATORDISENGAGE = 0.45;
    public static final int slideMaxHoriz = 1050;
    public static final int slideCushion = 150;
    public static final int armDownSlideOut = 350;
    public static final int armMax = 2000;
    public static final int armWithShoulderInFront = 2200;
    public static final int armCushion = 100;
    public static int motorRPM = 435;
    public static int slideTicksToInch = 78;
    public static double ARM_MULTIPLIER = 435.0 / (double) motorRPM; // was: mult
    //public static double armToDeg = 360.0/8192.0;
}
