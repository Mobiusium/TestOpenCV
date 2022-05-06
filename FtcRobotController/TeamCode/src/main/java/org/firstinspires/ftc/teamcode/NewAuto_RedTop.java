package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config //Disable if not using FTC Dashboard https://github.com/PinkToTheFuture/OpenCV_FreightFrenzy_2021-2022#opencv_freightfrenzy_2021-2022
@Autonomous(name="NewAuto_RedTop", group="Auto")

public class NewAuto_RedTop extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor rearLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor rearRightDrive = null;
    private DcMotor spinner = null;
    private DcMotor armMotor = null;
    private CRServo intake = null;
    private TouchSensor limit = null;

    private OpenCvCamera webcam;

    private static final int CAMERA_WIDTH  = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    private double CrLowerUpdate = 127;
    private double CbLowerUpdate = 0;
    private double CrUpperUpdate = 191;
    private double CbUpperUpdate = 100;

    public static double borderLeftX    = 0.0;   //fraction of pixels from the left side of the cam to skip
    public static double borderRightX   = 0.0;   //fraction of pixels from the right of the cam to skip
    public static double borderTopY     = 0.0;   //fraction of pixels from the top of the cam to skip
    public static double borderBottomY  = 0.0;   //fraction of pixels from the bottom of the cam to skip

    private double lowerruntime = 0;
    private double upperruntime = 0;

    // Pink Range                                      Y      Cr     Cb
//    public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 160.0, 100.0);
//    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 255.0);

    // Yellow Range
    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 100.0, 0.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 170.0, 120.0);

    //imu
    BNO055IMU imu;
    Orientation angles;

    @Override
    public void runOpMode()
    {
        //imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam1"), cameraMonitorViewId);
        //OpenCV Pipeline
        ContourPipeline myPipeline;
        webcam.setPipeline(myPipeline = new ContourPipeline(borderLeftX,borderRightX,borderTopY,borderBottomY));
        // Configuration of Pipeline
        myPipeline.configureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        myPipeline.configureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);
        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        // Only if you are using ftcdashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(webcam, 10);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive = hardwareMap.get(DcMotor.class, "left_drive1");
        rearLeftDrive  = hardwareMap.get(DcMotor.class, "left_drive2");
        frontRightDrive = hardwareMap.get(DcMotor.class, "right_drive1");
        rearRightDrive = hardwareMap.get(DcMotor.class, "right_drive2");
        armMotor = hardwareMap.get(DcMotor.class, "arm");
        spinner = hardwareMap.get(DcMotor.class, "spinner");
        intake = hardwareMap.get(CRServo.class, "intake");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotor.Direction.FORWARD);

//        TouchSensor limit = hardwareMap.get(TouchSensor.class, "limit");
        // DcMotor motor = hardwareMap.get(DcMotor.class, "Motor");

        telemetry.update();
        waitForStart();

        while (opModeIsActive())
        {

            myPipeline.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);
            if(myPipeline.error){
                telemetry.addData("Exception: ", myPipeline.debug);
            }
            // Only use this line of the code when you want to find the lower and upper values
            testing(myPipeline);

            telemetry.addData("RectArea: ", myPipeline.getRectArea());
            telemetry.update();

            if(myPipeline.getRectArea() > 2000){
                if(myPipeline.getRectMidpointX() > 400){
                    AUTONOMOUS_C();
                }
                else if(myPipeline.getRectMidpointX() > 200){
                    AUTONOMOUS_B();
                }
                else {
                    AUTONOMOUS_A();
                }
            }
        }
    }
    public void testing(ContourPipeline myPipeline){
        if(lowerruntime + 0.05 < getRuntime()){
            CrLowerUpdate += -gamepad1.left_stick_y;
            CbLowerUpdate += gamepad1.left_stick_x;
            lowerruntime = getRuntime();
        }
        if(upperruntime + 0.05 < getRuntime()){
            CrUpperUpdate += -gamepad1.right_stick_y;
            CbUpperUpdate += gamepad1.right_stick_x;
            upperruntime = getRuntime();
        }

        CrLowerUpdate = inValues(CrLowerUpdate, 0, 255);
        CrUpperUpdate = inValues(CrUpperUpdate, 0, 255);
        CbLowerUpdate = inValues(CbLowerUpdate, 0, 255);
        CbUpperUpdate = inValues(CbUpperUpdate, 0, 255);

        myPipeline.configureScalarLower(0.0, CrLowerUpdate, CbLowerUpdate);
        myPipeline.configureScalarUpper(255.0, CrUpperUpdate, CbUpperUpdate);

        telemetry.addData("lowerCr ", (int)CrLowerUpdate);
        telemetry.addData("lowerCb ", (int)CbLowerUpdate);
        telemetry.addData("UpperCr ", (int)CrUpperUpdate);
        telemetry.addData("UpperCb ", (int)CbUpperUpdate);
    }
    public Double inValues(double value, double min, double max){
        if(value < min){ value = min; }
        if(value > max){ value = max; }
        return value;
    }
    //left to right A B C Low Mid High
    public void AUTONOMOUS_A(){
        telemetry.addLine("Autonomous A");
        autoMoves("A");
    }
    public void AUTONOMOUS_B(){
        telemetry.addLine("Autonomous B");
        autoMoves("B");
    }
    public void AUTONOMOUS_C(){
        telemetry.addLine("Autonomous C");
        autoMoves("C");
    }

    public double getHeading(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    //Where procedure is happening
    public void autoMoves(String position){
        //Declare and Initiate a limit TouchSensor
        TouchSensor limit = hardwareMap.get(TouchSensor.class, "limit");
        //                                 distance / speed * 1000
        strafeRobot("left", 1100, 0.5);

        armToLevel(position);

        driveRobot(0.5,950);

        intake.setPower(-1);
        sleep(3000);
        intake.setPower(0);

        driveRobot(-0.5,1500);

        armMotor.setPower(-0.5);
        sleep(700);
        armMotor.setPower(0);

        strafeRobot("right",3000, 0.5);

        //arm down
        while(!limit.isPressed()&&opModeIsActive()){
            armMotor.setPower(0.5);
        }
        armMotor.setPower(0);

        sleep(10000);
        stop();
    }

    public void armToLevel(String position){
        if(position.equals("A")){
            armMotor.setPower(-0.5);
            sleep(600);
            armMotor.setPower(0);
        }
        if(position.equals("B")){
            armMotor.setPower(-0.5);
            sleep(1100);
            armMotor.setPower(0);
        }
        if(position.equals("C")){
            armMotor.setPower(-0.5);
            sleep(1700);
            armMotor.setPower(0);
        }
    }

    public void rotateToDegree(double degree) {
        resetIMU();
        double error, power, timeout = 10.0; //in seconds
        boolean timeStarted = false;
        resetStartTime();

        while (opModeIsActive() && getRuntime() <= timeout) {
            error = degree -getHeading();

            // one to one proportional control
//            power = correction(error, -45, 45, -1.0, 1.0);

            // skips low power
             if (error > 0){
                 power = correction(error, 0, 45.0, 0.1, 1.0);
             }
             else if (error < 0){
                 power = correction(error, -45.0, 0, -1.0, -0.1);
             }
             else {
                 power = 0;
             }

            telemetry.addData("Turning to:", degree);
            telemetry.addData("error: ", error);
            telemetry.addData("power: ", power);
            telemetry.addData("gyro: ", getHeading());
            telemetry.addData("runtime: ", getRuntime());
            telemetry.update();

            turnRobot(power);

            if (Math.abs(error) < 2 && !timeStarted) {
                resetStartTime();
                timeStarted = true;
                timeout = 1;
            }

        }
        brakeRobot();
    }

    public void resetIMU(){
        imu.initialize(imu.getParameters());
    }

    //use turn to degree instead of this
    public void turnRobot(double power){
        frontLeftDrive.setPower(-power);
        rearLeftDrive.setPower(-power);
        frontRightDrive.setPower(power);
        rearRightDrive.setPower(power);
    }

    //pow 0.5: 0.5 m/s
    public void driveRobot(double power, long time){
        frontLeftDrive.setPower(power);
        rearLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        rearRightDrive.setPower(power);
        sleep(time);
        brakeRobot();
    }

    //pow 0.5: 0.4 m/s
    public void strafeRobot(String direction,
                            long time, double power){
        if (direction.equals("right")){
            frontRightDrive.setPower(-power);
            rearRightDrive.setPower(power);
            frontLeftDrive.setPower(power);
            rearLeftDrive.setPower(-power);
            sleep(time);
            brakeRobot();
        }
        if (direction.equals("left")){
            frontRightDrive.setPower(power);
            rearRightDrive.setPower(-power);
            frontLeftDrive.setPower(-power);
            rearLeftDrive.setPower(power);
            sleep(time);
            brakeRobot();
        }
    }

    public void brakeRobot() {
        frontLeftDrive.setPower(0);
        rearLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearRightDrive.setPower(0);
        sleep(500);
    }

    public double correction(double value, double inMin, double inMax,
                             double outMin, double outMax) {
        return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

}
