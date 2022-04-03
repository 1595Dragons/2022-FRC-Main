package frc.robot.robotmap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class Shooter {

	// Shooter motor 1
	private static final int shooterMotor1ID = 30;
	public static final CANSparkMax shooterMotor1 = new CANSparkMax(shooterMotor1ID, CANSparkMaxLowLevel.MotorType.kBrushed);

	// Shooter motor 2
	private static final int shooterMotor2ID = 31;
	public static final CANSparkMax shooterMotor2 = new CANSparkMax(shooterMotor2ID, CANSparkMaxLowLevel.MotorType.kBrushless);

	// Constants for shooter speed.
	public static final double shootHigh = 0.75d; //73, 78
	public static final double shootLow = 0.4d; //FIXME
	public static final double autoShootTime = 3.0d; //FIXME

}
