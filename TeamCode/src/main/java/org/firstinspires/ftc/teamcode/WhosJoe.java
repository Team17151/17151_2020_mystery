package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU.Parameters;
import com.qualcomm.hardware.bosch.BNO055IMU.SensorMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

@TeleOp(
        name = "Joel",
        group = "use these"
)
public class WhosJoe extends LinearOpMode {
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

    public WhosJoe() {
    }

    public void runOpMode() {
        this.TR = (DcMotor)this.hardwareMap.dcMotor.get("TR");
        this.TL = (DcMotor)this.hardwareMap.dcMotor.get("TL");
        this.BL = (DcMotor)this.hardwareMap.dcMotor.get("BL");
        this.BR = (DcMotor)this.hardwareMap.dcMotor.get("BR");
        yoink1 = hardwareMap.dcMotor.get("yoink1");
        yoink2 = hardwareMap.dcMotor.get("yoink2");
        yeet1 = hardwareMap.dcMotor.get("yeet1");
        yeet2 = hardwareMap.dcMotor.get("yeet2");
        grab1 = hardwareMap.servo.get("grab1");
        grab2 = hardwareMap.servo.get("grab2");
        float gyro;
        float mod = 0.0f;
        double ra = 0.0D;
        double thoth = 0.0D;
        Parameters parameters = new Parameters();
        parameters.mode = SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        this.imu = (BNO055IMU)this.hardwareMap.get(BNO055IMU.class, "imu");
        this.imu.initialize(parameters);
        grab1.setPosition(1);
        grab2.setPosition(0);
        this.waitForStart();
        while(this.opModeIsActive()) {
            gyro = imu.getAngularOrientation().firstAngle - mod;
            if (this.gamepad1.x) {
                mod = this.imu.getAngularOrientation().firstAngle;
            }
            double cSs = Math.cos(-gyro * Math.PI / 180D) - Math.sin(-gyro * Math.PI / 180D);
            double cAs = Math.sin(-gyro * Math.PI / 180D) + Math.cos(-gyro * Math.PI / 180D);
            /*
            this.TR.setPower(cSs * (double)this.gamepad1.left_stick_y - cAs * (double)this.gamepad1.left_stick_x + (double)this.gamepad1.right_stick_x);
            this.TL.setPower(cAs * (double)this.gamepad1.left_stick_y + cSs * (double)this.gamepad1.left_stick_x - (double)this.gamepad1.right_stick_x);
            this.BL.setPower(cSs * (double)this.gamepad1.left_stick_y - cAs * (double)this.gamepad1.left_stick_x - (double)this.gamepad1.right_stick_x);
            this.BR.setPower(cAs * (double)this.gamepad1.left_stick_y + cSs * (double)this.gamepad1.left_stick_x + (double)this.gamepad1.right_stick_x);
            */
            TR.setPower(-((cSs * gamepad1.left_stick_y) + (cSs * gamepad1.left_stick_x) + gamepad1.right_stick_x));
            TL.setPower((cAs * gamepad1.left_stick_y) - (cAs * gamepad1.left_stick_x) - gamepad1.right_stick_x);
            BL.setPower((cSs * gamepad1.left_stick_y) + (cSs * gamepad1.left_stick_x) - gamepad1.right_stick_x);
            BR.setPower(-((cAs * gamepad1.left_stick_y) - (cAs * gamepad1.left_stick_x) + gamepad1.right_stick_x));
            yoink1.setPower(-(gamepad1.left_trigger + gamepad1.right_trigger));
            yoink2.setPower(-(gamepad1.left_trigger + gamepad1.right_trigger));
            yeet1.setPower(-gamepad1.right_trigger);
            yeet2.setPower(-gamepad1.right_trigger);
            if (gamepad1.left_bumper) {
                yoink1.setPower(1);
                yoink1.setPower(1);
            }
            if (gamepad1.a) {
                grab1.setPosition(1);
                grab2.setPosition(0);
            }
            if (gamepad1.b) {
                grab1.setPosition(0);
                grab2.setPosition(1);
            }
            this.telemetry.update();
        }

    }
}
