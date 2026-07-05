package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
        //hello coder im set up on a play set acting likeim coding but i actually cant code sike right im
        //jus acting nerdy and what not i didnt touch anything on your home page so be assured broooooooooo
        // idk what to do bro saveee me broooo hes reording idk whattt the hellllll im quite nervous
        // your keyboard is very ou

@TeleOp(name="Precision Tuner (Indexer Rev)", group="Test")
public class ShooterNormal extends OpMode {

    // ==========================================
    //              核心参数
    // ==========================================
    private double shooterUpRatio   = 0.80;
    private double shooterDownRatio = 1.00;

    private static final double STRAFE_CORRECTION = 1.1;
    private static final double TURN_SENSITIVITY = 1.0;

    // --- 硬件 ---
    private DcMotor frontLeftDrive, rearLeftDrive, frontRightDrive, rearRightDrive;
    private DcMotor shooterUpMotor, shooterDownMotor;
    private DcMotor intakeMotor, indexerMotor;

    // --- 变量 ---
    private double targetPower = 0.900;

    // 去抖
    private boolean lastDpadRight = false;
    private boolean lastDpadLeft  = false;
    private boolean lastDpadUp    = false;
    private boolean lastDpadDown  = false;

    private boolean lastLB = false;
    private boolean lastRB = false;

    private ElapsedTime buttonTimer = new ElapsedTime();
    private double lastButtonUpdateTime = 0;

    @Override
    public void init() {
        // 硬件映射
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left");
        rearLeftDrive   = hardwareMap.get(DcMotor.class, "rear_left");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
        rearRightDrive  = hardwareMap.get(DcMotor.class, "rear_right");

        shooterDownMotor = hardwareMap.get(DcMotor.class, "shooter_down");
        shooterUpMotor   = hardwareMap.get(DcMotor.class, "shooter_up");
        intakeMotor      = hardwareMap.get(DcMotor.class, "intake");
        indexerMotor     = hardwareMap.get(DcMotor.class, "indexer");

        shooterUpMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterDownMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotor.Direction.FORWARD);

        shooterUpMotor.setDirection(DcMotor.Direction.FORWARD);
        shooterDownMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        indexerMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterUpMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterDownMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        indexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Ready");
    }

    @Override
    public void loop() {

        double currentTime = buttonTimer.seconds();

        // ==========================================
        //      1. 粗调 +/- 0.01
        // ==========================================
        if (gamepad1.dpad_right) {
            if (!lastDpadRight || (currentTime - lastButtonUpdateTime > 0.5)) {
                targetPower += 0.01;
                lastButtonUpdateTime = currentTime;
            }
        } else if (gamepad1.dpad_left) {
            if (!lastDpadLeft || (currentTime - lastButtonUpdateTime > 0.5)) {
                targetPower -= 0.01;
                lastButtonUpdateTime = currentTime;
            }
        }

        // ==========================================
        //      2. 微调 +/- 0.001
        // ==========================================
        if (gamepad1.dpad_up && !lastDpadUp) targetPower += 0.001;
        else if (gamepad1.dpad_down && !lastDpadDown) targetPower -= 0.001;

        lastDpadRight = gamepad1.dpad_right;
        lastDpadLeft  = gamepad1.dpad_left;
        lastDpadUp    = gamepad1.dpad_up;
        lastDpadDown  = gamepad1.dpad_down;

        targetPower = Range.clip(targetPower, 0.0, 1.0);

        // ==========================================
        //      3. 肩键调上下轮比例
        // ==========================================
        if (gamepad1.left_bumper && !lastLB) {
            shooterUpRatio   -= 0.05;
            shooterDownRatio -= 0.05;
        }
        if (gamepad1.right_bumper && !lastRB) {
            shooterUpRatio   += 0.05;
            shooterDownRatio += 0.05;
        }

        shooterUpRatio   = Range.clip(shooterUpRatio,   0.0, 1.5);
        shooterDownRatio = Range.clip(shooterDownRatio, 0.0, 1.5);

        lastLB = gamepad1.left_bumper;
        lastRB = gamepad1.right_bumper;

        // ==========================================
        //      4. 射击输出（无电压补偿）
        // ==========================================
        if (gamepad1.right_trigger > 0.1) {
            shooterUpMotor.setPower(targetPower * shooterUpRatio);
            shooterDownMotor.setPower(targetPower * shooterDownRatio);
        } else {
            shooterUpMotor.setPower(0);
            shooterDownMotor.setPower(0);
        }

        // ==========================================
        //      5. Intake & Indexer
        // ==========================================
        if (gamepad1.a) intakeMotor.setPower(1.0);
        else if (gamepad1.b) intakeMotor.setPower(-1.0);
        else intakeMotor.setPower(0);

        if (gamepad1.x) indexerMotor.setPower(1.0);
        else if (gamepad1.y) indexerMotor.setPower(-1.0);
        else indexerMotor.setPower(0);

        driveBaseControl();

        // ==========================================
        //      6. Telemetry
        // ==========================================
        telemetry.addData("Target Power", "%.3f", targetPower);
        telemetry.addData("Up Ratio", "%.2f", shooterUpRatio);
        telemetry.addData("Down Ratio", "%.2f", shooterDownRatio);
        telemetry.update();
    }

    private void driveBaseControl() {
        double drive  = gamepad1.left_stick_x;
        double strafe = -gamepad1.left_stick_y * STRAFE_CORRECTION;
        double turn   = gamepad1.right_stick_x * TURN_SENSITIVITY;

        double fl = drive + strafe + turn;
        double fr = drive - strafe + turn;
        double rl = drive - strafe - turn;
        double rr = drive + strafe - turn;
        double max = Math.max(Math.abs(fl),
                Math.max(Math.abs(fr),
                        Math.max(Math.abs(rl), Math.abs(rr))));

        if (max > 1.0) {
            fl /= max; fr /= max; rl /= max; rr /= max;
        }

        frontLeftDrive.setPower(fl);
        rearLeftDrive.setPower(rl);
        frontRightDrive.setPower(fr);
        rearRightDrive.setPower(rr);
    }
}
