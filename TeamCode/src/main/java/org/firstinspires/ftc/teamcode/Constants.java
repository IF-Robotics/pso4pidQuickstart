package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;
import java.util.Arrays;

import ArmSpecific.GravityModelConstants;
import ArmSpecific.Hardware;
import ArmSpecific.SystemConstants;
import ArmSpecific.pso4Arms;
import CommonUtilities.AngleRange;
import CommonUtilities.PIDFParams;
import CommonUtilities.PIDFcontroller;

@Config
public class Constants {

    //CONFIGURATIONS - do before running anything

    //todo Assign Your Motor Specs, direction, and CONFIG Name
    public final Hardware.Motor motor = Hardware.YellowJacket.RPM84; // for geared motors new Hardware.Motor(RPM,EncTicksPerRev,StallTorque,GearRatio);
    public final DcMotorSimple.Direction motorDirection = DcMotorSimple.Direction.REVERSE;
    public final String motorName = "shoulder";


    //todo provide the angles (in radians) that your arm can run to when testing (larger range the better)
    public final AngleRange testingAngle = new AngleRange(0.0, PI);
    //todo provide angles (in radians) that present as obstacles to the system. If none set to null
    public final AngleRange obstacle = new AngleRange(-0.1 * PI, -0.3 * PI); // = null


    //TESTING

    //todo change from FRICTION OPMODE results
    public static double frictionRPM = 74.9;
    public static double inertiaValue = 1.170751047881278;


    //todo change from Gravity OPMODE & Desmos Graph
    public static double gravityA = -8.74869;
    public static double gravityB = 1.59221;
    public static double gravityK = 21.6828;

    //todo name the angles you want your arm to run to (The simulator will generate PIDF coefficients for each)

    static double stationaryAngle = Math.toRadians(3.0);

    public static final ArrayList<AngleRange> angleRanges = new ArrayList<AngleRange>() {{
        add(new AngleRange(stationaryAngle, PI*.5));
        add(new AngleRange(PI*.5, -PI*.9));
        add(new AngleRange(-PI*.9, PI*.5));
        add(new AngleRange(PI*.5, stationaryAngle));
    }};

    //todo LAST STEP - RUN the test in the TEST MODULE -> TeamCode/src/test/java/org.firstinspires.ftc.teamcode/FindConstants.java

    public static ArrayList<PIDFParams> params = new ArrayList<>(Arrays.asList(
            new PIDFParams(2.285019340839641, 0.4386279973226315, 0.30841127892035985, 0.14487563709714993),
            new PIDFParams(3.569998446313589, 0.8152903499711774, 0.41822071381802867, 1.2682980274992102),
            new PIDFParams(3.017739632654526, 0.7573771973745118, 0.3547940131426662, 0.5146750002011186),
            new PIDFParams(3.075062305773028, 0.14718362175735816, 0.32082493355821445, 0.8737486665205733)
    ));


    SystemConstants constant = new SystemConstants(
            frictionRPM,
            motor,
            new GravityModelConstants(gravityA, gravityB, gravityK),
            inertiaValue
    );
    public pso4Arms sim = new pso4Arms(constant, angleRanges, 1.2, obstacle, 3.5);
    public static boolean gravityRecord = false;
    public static boolean gravityDisplayDataPoints = false;
    public static double gravityMotorPower = 0.0;
    public PIDFcontroller pidfController = new PIDFcontroller(
            new PIDFParams(0.0, 0.0, 0.0, 0.0),
            motor,
            obstacle,
            Math.toRadians(3.0)
    );

}
