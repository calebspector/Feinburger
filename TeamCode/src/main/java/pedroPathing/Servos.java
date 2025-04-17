package pedroPathing;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Servos {
    public Servo sweep, up, hang, wrist, shoulder, shoulder2, claw, actuator, pullUp;

    public Servos(HardwareMap hardwareMap) {
        sweep = hardwareMap.get(Servo.class, "sweep");
        up = hardwareMap.get(Servo.class, "up");
        hang = hardwareMap.get(Servo.class, "hang");
        wrist = hardwareMap.get(Servo.class, "wrist");
        shoulder = hardwareMap.get(Servo.class, "shoulder");
        shoulder2 = hardwareMap.get(Servo.class, "shoulder2");
        claw = hardwareMap.get(Servo.class, "claw");
        actuator = hardwareMap.get(Servo.class, "actuator");
        pullUp = hardwareMap.get(Servo.class, "pullUp");
    }
}
