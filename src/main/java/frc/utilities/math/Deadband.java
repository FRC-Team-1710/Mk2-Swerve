package frc.utilities.math;

public class Deadband {
    public static double deadband(double input) {
		return deadband(input, 0.025);
	}

	public static double deadband(double input, double buffer) {
		if (Math.abs(input) < buffer) return 0;
		return input;
	}
}
