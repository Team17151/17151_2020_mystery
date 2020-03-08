package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU.Parameters;
import com.qualcomm.hardware.bosch.BNO055IMU.SensorMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.VoltageSensor;

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
    private Servo clawboi;
    private BNO055IMU imu;
    private ColorSensor veryvery;
    private VoltageSensor voltageSensor;

    public void runOpMode() {
        this.TR = this.hardwareMap.dcMotor.get("TR");
        this.TL = this.hardwareMap.dcMotor.get("TL");
        this.BL = this.hardwareMap.dcMotor.get("BL");
        this.BR = this.hardwareMap.dcMotor.get("BR");
//        voltageSensor = hardwareMap.voltageSensor.get("");
        yoink1 = hardwareMap.dcMotor.get("yoink1");
        yoink2 = hardwareMap.dcMotor.get("yoink2");
        yeet1 = hardwareMap.dcMotor.get("yeet1");
        yeet2 = hardwareMap.dcMotor.get("yeet2");
        grab1 = hardwareMap.servo.get("grab1");
        grab2 = hardwareMap.servo.get("grab2");
        clawboi = hardwareMap.servo.get("clawboi");
        veryvery = hardwareMap.get(ColorSensor.class, "veryvery");
        //Assign the voltage sensor object to a motor controller
        //I don't know why this is how the API is designed... but it is...
//        voltageSensor = hardwareMap.voltageSensor.get("TR");
        TL.setDirection(Direction.REVERSE);
        BL.setDirection(Direction.REVERSE);
        double gyro;
        double tong;
        double mod = 0D;
        double pTime = 0D;
        double oTime;
        double xpeed = 0D;
        double zpeed = 0D;
        double aXel;
        double aZel;
        double cX = 0D;
        double cZ = 0D;
        float[] hsv = new float[3];
        double[][] stuf = new double[3][2];
        String[] stufstr = {"xlr8 ","fast ","true "};
        Parameters parameters = new Parameters();
        parameters.mode = SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        this.imu = hardwareMap.get(BNO055IMU.class, "imu");
        this.imu.initialize(parameters);
        grab1.setPosition(1);
        grab2.setPosition(0);
        this.waitForStart();
        pTime = time;
        while(this.opModeIsActive()) {
            tong = 1;
            gyro = imu.getAngularOrientation().firstAngle - mod;
            if (this.gamepad1.x) {
                mod = this.imu.getAngularOrientation().firstAngle;
            }
            if (gamepad1.left_stick_button) {
                tong = 0.3;
            }
            oTime = time - pTime;
            pTime = time;
            aXel = ((imu.getLinearAcceleration().xAccel * Math.cos(-gyro * Math.PI / 180)) - (Math.sin(-gyro * Math.PI / 180) * imu.getLinearAcceleration().zAccel));
            aZel = ((imu.getLinearAcceleration().zAccel * Math.cos(-gyro * Math.PI / 180)) + (Math.sin(-gyro * Math.PI / 180) * imu.getLinearAcceleration().xAccel));
            xpeed += oTime * aXel;
            zpeed += oTime * aZel;
            cX += oTime * xpeed;
            cZ += oTime * zpeed;
            double cSs = Math.cos(-gyro * Math.PI / 180D) - Math.sin(-gyro * Math.PI / 180D);
            double cAs = Math.sin(-gyro * Math.PI / 180D) + Math.cos(-gyro * Math.PI / 180D);
            TR.setPower(tong * ((cSs * gamepad1.left_stick_y) + (-cAs * gamepad1.left_stick_x) + gamepad1.right_stick_x));
            TL.setPower(tong * ((cAs * gamepad1.left_stick_y) + (cSs * gamepad1.left_stick_x) - gamepad1.right_stick_x));
            BR.setPower(tong * ((cAs * gamepad1.left_stick_y) + (cSs * gamepad1.left_stick_x) + gamepad1.right_stick_x));
            BL.setPower(tong * ((cSs * gamepad1.left_stick_y) + (-cAs * gamepad1.left_stick_x) - gamepad1.right_stick_x));
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
            if (gamepad1.left_bumper) {
                clawboi.setPosition(0);
            }
            if (gamepad1.right_bumper) {
                clawboi.setPosition(.6);
            }
            stuf[0][0] = aXel;
            stuf[0][1] = aZel;
            stuf[1][0] = xpeed;
            stuf[1][1] = zpeed;
            stuf[2][0] = cX;
            stuf[2][1] = cZ;
            for (int x = 0; x <= 2; x++) {
                for (int y = 0; y <= 1; y++) {
                    telemetry.addData(stufstr[x] + Integer.toString(y),stuf[x][y]);
                }
            }
            telemetry.addData("fps", 1 / oTime);
            //print voltage
//            telemetry.addData("Current Voltage", voltageSensor.getVoltage());
            telemetry.addData("colors", veryvery.red() + " " + veryvery.blue());
            telemetry.update();
        }

    }
}