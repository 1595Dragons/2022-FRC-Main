package frc.robot.robotmap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class Indexer {

	// Indexer motor
	private static final int indexerMotorID = 33;
	public static final CANSparkMax indexerMotor = new CANSparkMax(indexerMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);

	//private static final int intakeSensorID = 0;
	//private static final int ballOneSensorID = 1;
	//private static final int ballTwoSensorID = 2;
	public static final double indexSpeedForward = -0.3d; //FIXME
	public static final double indexSpeedSimple = 0.3d; //FIXME
	public static final double indexSpeedSlow = 0.1d;
	public static final double indexWrongBallOut = -0.6d;
	public static final double indexSpeedSimpleBack = -0.2d;
	public static final double readyIndexForShoot = 0.4d;

}
