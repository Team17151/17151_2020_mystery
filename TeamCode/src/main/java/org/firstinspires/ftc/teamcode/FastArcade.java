package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU.Parameters;
import com.qualcomm.hardware.bosch.BNO055IMU.SensorMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

@TeleOp(
        name = "Shuttle",
        group = "use these"
)
public class FastArcade extends LinearOpMode {
    private DcMotor TR;
    private DcMotor TL;
    private DcMotor BL;
    private DcMotor BR;
    private DcMotor yoink1;
    private DcMotor yoink2;
    private DcMotor yeet1;
    private DcMotor yeet2;
    private Servo grab1;
    private Servo grab2;
    private BNO055IMU imu;

    public void runOpMode() {
        this.TR = this.hardwareMap.dcMotor.get("TR");
        this.TL = this.hardwareMap.dcMotor.get("TL");
        this.BL = this.hardwareMap.dcMotor.get("BL");
        this.BR = this.hardwareMap.dcMotor.get("BR");
        yoink1 = hardwareMap.dcMotor.get("yoink1");
        yoink2 = hardwareMap.dcMotor.get("yoink2");
        yeet1 = hardwareMap.dcMotor.get("yeet1");
        yeet2 = hardwareMap.dcMotor.get("yeet2");
        grab1 = hardwareMap.servo.get("grab1");
        grab2 = hardwareMap.servo.get("grab2");
        TL.setDirection(Direction.REVERSE);
        BL.setDirection(Direction.REVERSE);
        double gyro;
        double mod = 0D;
        double pTime = 0D;
        double oTime;
        double speed = 0D;
        Parameters parameters = new Parameters();
        parameters.mode = SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.MILLI_EARTH_GRAVITY;
        this.imu = hardwareMap.get(BNO055IMU.class, "imu");
        this.imu.initialize(parameters);
        grab1.setPosition(1);
        grab2.setPosition(0);
        this.waitForStart();
        while(this.opModeIsActive()) {
            gyro = imu.getAngularOrientation().firstAngle - mod;
            if (this.gamepad1.x) {
                mod = this.imu.getAngularOrientation().firstAngle;
            }
            oTime = time - pTime;
            pTime = time;
            speed += oTime * imu.getLinearAcceleration().xAccel;
            double cSs = Math.cos(-gyro * Math.PI / 180D) - Math.sin(-gyro * Math.PI / 180D);
            double cAs = Math.sin(-gyro * Math.PI / 180D) + Math.cos(-gyro * Math.PI / 180D);
            TR.setPower((cSs * gamepad1.left_stick_y) + (-cAs * gamepad1.left_stick_x) + gamepad1.right_stick_x);
            TL.setPower((cAs * gamepad1.left_stick_y) + (cSs * gamepad1.left_stick_x) - gamepad1.right_stick_x);
            BR.setPower((cAs * gamepad1.left_stick_y) + (cSs * gamepad1.left_stick_x) + gamepad1.right_stick_x);
            BL.setPower((cSs * gamepad1.left_stick_y) + (-cAs * gamepad1.left_stick_x) - gamepad1.right_stick_x);
            yoink1.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
            yoink2.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
            yeet1.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
            yeet2.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
            if (gamepad1.a) {
                grab1.setPosition(1);
                grab2.setPosition(0);
            }
            if (gamepad1.b) {
                grab1.setPosition(0);
                grab2.setPosition(1);
            }
            telemetry.addData("thing1", imu.getLinearAcceleration());
            telemetry.addData("thing2", speed);
            telemetry.addData("fps", 1 / oTime);
            telemetry.update();
        }

    }
}
