package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU.Parameters;
import com.qualcomm.hardware.bosch.BNO055IMU.SensorMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Autonomous(name = "newAutoBlu")
public class NewAutoBlu extends LinearOpMode {
    private DcMotor TR;
    private DcMotor TL;
    private DcMotor BL;
    private DcMotor BR;
    private Servo grab1;
    private Servo grab2;
    private BNO055IMU imu;
    public void runOpMode() {
        this.TR = this.hardwareMap.dcMotor.get("TR");
        this.TL = this.hardwareMap.dcMotor.get("TL");
        this.BL = this.hardwareMap.dcMotor.get("BL");
        this.BR = this.hardwareMap.dcMotor.get("BR");
        grab1 = hardwareMap.servo.get("grab1");
        grab2 = hardwareMap.servo.get("grab2");
        Parameters parameters = new Parameters();
        parameters.mode = SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        this.imu = hardwareMap.get(BNO055IMU.class, "imu");
        this.imu.initialize(parameters);
        grab1.setPosition(1);
        grab2.setPosition(0);
        this.TR.setDirection(Direction.REVERSE);
        this.BR.setDirection(Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            move(-1.6D,.2D,0D,3D,.3D);
            TR.setPower(0);
            TL.setPower(0);
            BR.setPower(0);
            BL.setPower(0);
        }
    }
    public void move(Double x, Double z, Double Trot, Double MaxT, Double MaxPower) {
        double stime = time;
        double ptime = time;
        double otime;
        double xpeed = 0D;
        double zpeed = 0D;
        double cX = 0D;
        double cZ = 0D;
        double aXel;
        double aZel;
        double gyro;
        double gyyg;
        double sg = -imu.getAngularOrientation().firstAngle;
        while (time < stime + MaxT) {
             gyro = -imu.getAngularOrientation().firstAngle - sg;
             otime = time - ptime;
             ptime = time;
             aXel = ((imu.getLinearAcceleration().xAccel * Math.cos(gyro * Math.PI / 180)) - (Math.sin(gyro * Math.PI / 180) * imu.getLinearAcceleration().zAccel));
             aZel = ((imu.getLinearAcceleration().zAccel * Math.cos(gyro * Math.PI / 180)) + (Math.sin(gyro * Math.PI / 180) * imu.getLinearAcceleration().xAccel));
             xpeed += otime * aXel;
             zpeed += otime * aZel;
             cX += otime * xpeed;
             cZ += otime * zpeed;
             double cSs = Math.cos(gyro * Math.PI / 180D) - Math.sin(gyro * Math.PI / 180D);
             double cAs = Math.sin(gyro * Math.PI / 180D) + Math.cos(gyro * Math.PI / 180D);
             TR.setPower(MaxPower * (cSs * (x - cX)) + (cAs * (z - cZ)) + (-(Trot - gyro)/270));
             TL.setPower(MaxPower * (cAs * (x - cX)) + (-cSs * (z - cZ)) + ((Trot - gyro)/270));
             BL.setPower(MaxPower * (cSs * (x - cX)) + (cAs * (z - cZ)) + ((Trot - gyro)/270));
             BR.setPower(MaxPower * (cAs * (x - cX)) + (-cSs * (z - cZ)) + (-(Trot - gyro)/270));
             telemetry.addData("bubba", "\n" + Double.toString(cX) + "\n" + Double.toString(cZ));
             telemetry.update();
        }
    }
    public void move(Double x, Double z, Double Trot, Double MaxT) {
        move(x,z, Trot, MaxT,1D);
    }
}
