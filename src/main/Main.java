package main;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import org.apache.commons.math3.fitting.PolynomialCurveFitter;
import org.apache.commons.math3.fitting.WeightedObservedPoints;

import tracereplay.DirectoryWalker;
import tracereplay.PreProcess;
import tracereplay.ReadWriteTrace;
import tracereplay.RealTimeBehaviorDetector;
import tracereplay.RotationMatrix;
import utility.Constants;
import utility.Log;
import utility.Trace;
import database.SqliteAccess;

public class Main {

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		Main main = new Main();
		main.start();
	}
	
	
	private static String input = "/home/lkang/Dropbox/projects/drivesense/data/lei_db/";
	private static String output = "/home/lkang/Dropbox/projects/drivesense/data/lei_dat/";
	
	public void start() {
		List<String> files = DirectoryWalker.getFileNames(input);
		for(String file: files) {
			String name = file.substring(0, 13);
			String ipath = input.concat(file);
			long start = Long.valueOf(name);
			String opath = output.concat(name);
			
			DirectoryWalker.createFolder(opath);
			//processTrip(ipath, start, opath);
			flatRoadDetector(ipath, start, opath);
			break;
		}
	}
	
	public static List<Trace> loadOBDSpeed(String path, long start) {		
		List<Trace> speed = new ArrayList<Trace>();
		List<Trace> obdspeed = SqliteAccess.loadOBDData(path, start, Trace.SPEED);
		for(int i = 0; i < obdspeed.size() - 1; ++i) {
			Trace cur = obdspeed.get(i);
			Trace next = obdspeed.get(i + 1);
			long diffTime = next.time - cur.time;
			double diffSpeed = next.values[0] - cur.values[0];
			double acce = diffSpeed * Constants.kKmPHToMeterPS / ((double)diffTime/1000.0);
			Trace nTrace = new Trace(2);
			nTrace.time = cur.time;
			nTrace.type = cur.type;
			nTrace.values[0] = cur.values[0] * Constants.kKmPHToMeterPS;
			nTrace.values[1] = acce;
			speed.add(nTrace);
		}
		return speed;
	}
	
	private static void calculateGravity(List<Trace> accelerometer) {
		for(int i = 0; i < accelerometer.size(); ++i) {
			double gravity = 0.0;
			for(int j = 0; j < accelerometer.get(j).dim; ++j) {
				gravity += Math.pow(accelerometer.get(j).values[j], 2.0);
			}
			Log.log(Math.sqrt(gravity));
		}
	}
	
	
	public static double linear_correlation(List<Trace> input, int x, int y) {
		double corr = 0.0;
		int sz = input.size();
		double average_x = 0.0;
		double average_y = 0.0;
		for(int i = 0 ; i < sz; ++i) {
			average_x += input.get(i).values[x];
			average_y += input.get(i).values[y];
		}
		average_x /= sz;
		average_y /= sz;
		
		double upper = 0.0;
		double m_x = 0.0, m_y = 0.0;
		for(int i = 0 ; i < sz; ++i) {
			double tmpx = input.get(i).values[x];
			double tmpy = input.get(i).values[y];
			upper += (tmpx - average_x) * (tmpy - average_y);
			m_x += (tmpx - average_x) * (tmpx - average_x);
			m_y += (tmpy - average_y) * (tmpy - average_y);
		}
		if(m_x*m_y ==0 || m_x*m_y != m_x*m_y) corr = 1;
		else corr = upper / Math.sqrt(m_x * m_y);
		
		return corr;
	}
	
	//important
	public static List<Trace> traceReplay(List<List<Trace> > input) {
		int num = input.size();
		int index[] = new int[num];
		for(int i = 0; i < num; ++i) {
			index[i] = 0;
		}
		RealTimeBehaviorDetector detector = new RealTimeBehaviorDetector();
		while(true) {
			int cur = -1;
			long time = Long.MAX_VALUE;
			for(int i = 0; i < num; ++i) {
				int j = index[i];
				if(j >= input.get(i).size()) {
					continue;
				}
				Trace trace = input.get(i).get(j);
				if(trace.time < time) {
					time = trace.time;
					cur = i;
				}
			}
			if(-1 == cur) {
				break;
			}
			detector.processTrace(input.get(cur).get(index[cur]));
			index[cur] ++;
		}
		return detector.projected_accelerometer;
	}
	
	
	private static Trace rotate(Trace raw_tr, double[] rM) {
		Trace calculated_tr = new Trace();
		calculated_tr.time = raw_tr.time;
		double x, y, z;
		x = raw_tr.values[0];
		y = raw_tr.values[1];
		z = raw_tr.values[2];

		calculated_tr.values[0] = x * rM[0] + y * rM[1] + z * rM[2];
		calculated_tr.values[1] = x * rM[3] + y * rM[4] + z * rM[5];
		calculated_tr.values[2] = x * rM[6] + y * rM[7] + z * rM[8];

		return calculated_tr;
	}

	

	private static boolean isTurning(List<Trace> wingyro) {
		Trace sum = new Trace(3);
		sum.time = wingyro.get(0).time;
		for(int i = 0; i < wingyro.size(); ++i) {
			Trace cur = wingyro.get(i);
			for(int j = 0; j < sum.dim; ++j) {
				sum.values[j] += cur.values[j];
			}
		}
		boolean turnning = false;
		for(int j = 0; j < sum.dim; ++j) {
			sum.values[j] /= wingyro.size();
			if(Math.abs(sum.values[j]) > 0.05) {
				turnning = true;
			}
		}
		return turnning;
	}

	public static void getRotationMatrixFromVector(double[] R, double[] rotationVector) {

        double q0;
        double q1 = rotationVector[0];
        double q2 = rotationVector[1];
        double q3 = rotationVector[2];
        q0 = 1 - q1*q1 - q2*q2 - q3*q3;
        q0 = (q0 > 0) ? (double)Math.sqrt(q0) : 0;

        double sq_q1 = 2 * q1 * q1;
        double sq_q2 = 2 * q2 * q2;
        double sq_q3 = 2 * q3 * q3;
        double q1_q2 = 2 * q1 * q2;
        double q3_q0 = 2 * q3 * q0;
        double q1_q3 = 2 * q1 * q3;
        double q2_q0 = 2 * q2 * q0;
        double q2_q3 = 2 * q2 * q3;
        double q1_q0 = 2 * q1 * q0;           
        R[0] = 1 - sq_q2 - sq_q3;
        R[1] = q1_q2 - q3_q0;
        R[2] = q1_q3 + q2_q0;

        R[3] = q1_q2 + q3_q0;
        R[4] = 1 - sq_q1 - sq_q3;
        R[5] = q2_q3 - q1_q0;

        R[6] = q1_q3 - q2_q0;
        R[7] = q2_q3 + q1_q0;
        R[8] = 1 - sq_q1 - sq_q2;   
    }
	
	
	private static final double NS2S = 1.0f / 1000000000.0f;
	private final double[] deltaRotationVector = new double[4];
	private double timestamp = 0;
	public static final double EPSILON = 0.000000001f;
	
	public Trace onGyroscopeChanged(Trace gyro) {
	  // This timestep's delta rotation to be multiplied by the current rotation
	  // after computing it from the gyro sample data.
		if (timestamp != 0) {
			final double dT = (gyro.time - timestamp) * NS2S;
			// Axis of the rotation sample, not normalized yet.
			double axisX = gyro.values[0];
			double axisY = gyro.values[1];
			double axisZ = gyro.values[2];

			// Calculate the angular speed of the sample
			double omegaMagnitude = Math.sqrt(axisX*axisX + axisY*axisY + axisZ*axisZ);
			
			// Normalize the rotation vector if it's big enough to get the axis
			// (that is, EPSILON should represent your maximum allowable margin of error)
			if (omegaMagnitude > EPSILON) {
				axisX /= omegaMagnitude;
				axisY /= omegaMagnitude;
				axisZ /= omegaMagnitude;
			}
			double thetaOverTwo = omegaMagnitude * dT / 2.0f;
			double sinThetaOverTwo = Math.sin(thetaOverTwo);
			double cosThetaOverTwo = Math.cos(thetaOverTwo);
			deltaRotationVector[0] = sinThetaOverTwo * axisX;
			deltaRotationVector[1] = sinThetaOverTwo * axisY;
			deltaRotationVector[2] = sinThetaOverTwo * axisZ;
			deltaRotationVector[3] = cosThetaOverTwo;
		}
		timestamp = gyro.time;
		Trace tr = new Trace(9);
		tr.time = gyro.time;
		
		Log.log(deltaRotationVector[2]);
		
		getRotationMatrixFromVector(tr.values, deltaRotationVector);
		return tr;
	}
	public void flatRoadDetector(String path, long start, String opath) {
		List<Trace> speed = PreProcess.interpolate(loadOBDSpeed(path, start), 1.0);
		List<Trace> accelerometer = SqliteAccess.loadSensorData(path, start, Trace.ACCELEROMETER);
		List<Trace> gyroscope = SqliteAccess.loadSensorData(path, start, Trace.GYROSCOPE);
		List<Trace> rotation_matrix = SqliteAccess.loadSensorData(path, start, Trace.ROTATION_MATRIX);
		List<Trace> gps = SqliteAccess.loadSensorData(path, start, Trace.GPS);
		
		
		List<Trace> smoothed_accelerometer = PreProcess.exponentialMovingAverage(accelerometer);
		List<Trace> smoothed_gyroscope = PreProcess.exponentialMovingAverage(gyroscope);
		List<Trace> smoothed_rm = PreProcess.exponentialMovingAverage(rotation_matrix);

		
		List<Trace> subacce = PreProcess.extractSubList(smoothed_accelerometer, 130*1000, 200*1000);
		List<Trace> subgyro = PreProcess.extractSubList(smoothed_gyroscope, 130*1000, 200*1000);
		List<Trace> subrm = PreProcess.extractSubList(rotation_matrix, 130*1000, 135*1000);
		Trace rm = PreProcess.getAverage(subrm);
		
		Trace gyro_drift = PreProcess.getAverage(subgyro.subList(0, 10));
		Log.log(gyro_drift);
		
		List<Trace> res = new ArrayList<Trace>();
		List<Trace> direction = new ArrayList<Trace>();
		boolean begin = false, over = false;
		for(Trace trace: subacce) {
			Trace cur = rotate(trace, rm.values);
			double tmp = Math.sqrt(Math.pow(cur.values[0], 2.0) + Math.pow(cur.values[1], 2.0)); 
			if(!over && tmp > 0.6) {
				direction.add(cur);
				begin = true;
			}
			if(!over && tmp < 0.6 && begin == true) {
				over = true;
			}
			res.add(cur);
		}
		
		
		Trace test = PreProcess.getAverage(direction);
		Trace unit = getUnitVector(test);
		Trace yaxe = new Trace(3);
		yaxe.setValues(0.0, 1.0, 0.0);
		Trace xyrm = rmBetweenVectors(unit, yaxe);
		
		Log.log(xyrm);
		
		List<Trace> subprojected = new ArrayList<Trace>();
		for(Trace trace: res) {
			Trace ntr = rotate(trace, xyrm.values);
			subprojected.add(ntr);
		}
		
		List<Trace> projectedgyro = new ArrayList<Trace>();
		for(Trace trace: subgyro) {
			Log.log(trace);
			for(int i = 0; i < 3; ++i) {
				trace.values[i] -= gyro_drift.values[i];
			}
			Trace rot = rotate(trace, rm.values);
			Trace cur = rotate(rot, xyrm.values);
			projectedgyro.add(cur);
		}
		
		
		Log.log(unit);
		curveFit(res);
		
		
		ReadWriteTrace.writeFile(speed, opath + "/speed.dat");
		ReadWriteTrace.writeFile(res, opath + "/fr_rotated_accelerometer.dat");
		ReadWriteTrace.writeFile(subprojected, opath + "/fr_projected_accelerometer.dat");
		ReadWriteTrace.writeFile(projectedgyro, opath + "/fr_projected_gyroscope.dat");
		
		
		List<Trace> deltaRM = new ArrayList<Trace>();
		List<Trace> accumulatedGyro = new ArrayList<Trace>();
		Trace sum = new Trace(3);
		for(Trace trace: projectedgyro) {
			for(int i = 0; i < trace.dim; ++i) {
				sum.values[i] += trace.values[i];
				
			}
			Trace ntr = new Trace(3);
			ntr.copyTrace(sum);
			ntr.time = trace.time;
			accumulatedGyro.add(ntr);
		}
		List<Trace> verticalAligned = new ArrayList<Trace>();
		for(Trace trace: subprojected) {
			long time = trace.time;
			Trace delta = PreProcess.getTraceAt(deltaRM, time);
			if(delta == null) continue;
			Trace vrot = rotate(trace, delta.values);
			verticalAligned.add(vrot);
		}
		ReadWriteTrace.writeFile(verticalAligned, opath + "/fr_valigned_accelerometer.dat");
		ReadWriteTrace.writeFile(accumulatedGyro, opath + "/fr_accumulated_gyroscope.dat");

		/*
		List<Trace> corr_gyro = calculateCorrelation(projectedgyro);
		ReadWriteTrace.writeFile(corr_gyro, opath + "/fr_correlation_projected_gyro.dat");
		*/
	}
	
	private List<Trace> calculateCorrelation(List<Trace> traces) {
		List<Trace> correlations = new ArrayList<Trace>();
		int wnd = 20;
		List<Trace> window = new LinkedList<Trace>();
		for(int i = 0; i < traces.size(); ++i) {
			Trace tr = new Trace(4);
			tr.time = traces.get(i).time;
			System.arraycopy(traces.get(i).values, 0, tr.values, 0, 3);
			tr.values[3] = Math.sqrt(Math.pow(tr.values[0], tr.values[1]));
			window.add(tr);
			if(window.size() >= wnd) {
				window.remove(0);
				Trace trace = new Trace(3);
				trace.time = traces.get(i).time;
				trace.values[0] = linear_correlation(window, 0, 1);
				trace.values[1] = linear_correlation(window, 1, 2);
				trace.values[2] = linear_correlation(window, 0, 2);		
				correlations.add(trace);
			}
		}
		return correlations;
	}
	
	private static Trace getUnitVector(Trace input) {
		Trace res = new Trace(3);
		double sum = Math.sqrt(Math.pow(input.values[0], 2.0) + Math.pow(input.values[1], 2.0));
		res.setValues(input.values[0]/sum, input.values[1]/sum, 0.0);
		return res;
	}
	
	private static Trace rmBetweenVectors(Trace v0, Trace v1) {
		Trace res = new Trace(9);
		double cos_theta = v0.values[0] * v1.values[0] + v0.values[1] * v1.values[1];
		double sin_theta = v0.values[0] * v1.values[1] - v0.values[1] * v1.values[0];
		res.values[0] = cos_theta;
		res.values[1] = - sin_theta;
		res.values[2] = 0.0;
		res.values[3] = sin_theta;
		res.values[4] = cos_theta;
		for(int i = 5; i < 8; ++i) {
			res.values[i] = 0.0;
		}
		res.values[8] = 1.0;
		return res;
	}
	
	private static void curveFit(List<Trace> acce) {
		final WeightedObservedPoints obs = new WeightedObservedPoints();
		for(Trace trace: acce) {
			double x = trace.values[0];
			double y = trace.values[1];
			obs.add(x, y);
		}
		// Instantiate a third-degree polynomial fitter.
		final PolynomialCurveFitter fitter = PolynomialCurveFitter.create(1);
		// Retrieve fitted parameters (coefficients of the polynomial function).
		final double[] coeff = fitter.fit(obs.toList());
		Log.log(coeff[0], coeff[1]);
	}


	/*
	public void onGyroscopeSensorChanged(float[] gyroscope, long timestamp)
	{
		// don't start until first accelerometer/magnetometer orientation has
		// been acquired
		if (!hasInitialOrientation)
		{
			return;
		}

		// This timestep's delta rotation to be multiplied by the current
		// rotation after computing it from the gyro sample data.
		if (timestampOld != 0 && stateInitialized)
		{
			final float dT = (timestamp - timestampOld) * NS2S;

			// Axis of the rotation sample, not normalized yet.
			float axisX = gyroscope[0];
			float axisY = gyroscope[1];
			float axisZ = gyroscope[2];

			// Calculate the angular speed of the sample
			float omegaMagnitude = (float) Math.sqrt(axisX * axisX + axisY
					* axisY + axisZ * axisZ);

			// Normalize the rotation vector if it's big enough to get the axis
			if (omegaMagnitude > EPSILON)
			{
				axisX /= omegaMagnitude;
				axisY /= omegaMagnitude;
				axisZ /= omegaMagnitude;
			}

			// Integrate around this axis with the angular speed by the timestep
			// in order to get a delta rotation from this sample over the
			// timestep. We will convert this axis-angle representation of the
			// delta rotation into a quaternion before turning it into the
			// rotation matrix.
			float thetaOverTwo = omegaMagnitude * dT / 2.0f;

			float sinThetaOverTwo = (float) Math.sin(thetaOverTwo);
			float cosThetaOverTwo = (float) Math.cos(thetaOverTwo);

			deltaRotationVector[0] = sinThetaOverTwo * axisX;
			deltaRotationVector[1] = sinThetaOverTwo * axisY;
			deltaRotationVector[2] = sinThetaOverTwo * axisZ;
			deltaRotationVector[3] = cosThetaOverTwo;

			SensorManager.getRotationMatrixFromVector(deltaRotationMatrix,
					deltaRotationVector);

			currentRotationMatrix = matrixMultiplication(currentRotationMatrix,
					deltaRotationMatrix);

			SensorManager.getOrientation(currentRotationMatrix,
					gyroscopeOrientation);

			// values[0]: azimuth, rotation around the Z axis.
			// values[1]: pitch, rotation around the X axis.
			// values[2]: roll, rotation around the Y axis.

			// Find the gravity component of the X-axis
			// = g*-cos(pitch)*sin(roll);
			components[0] = (float) (SensorManager.GRAVITY_EARTH
					* -Math.cos(gyroscopeOrientation[1]) * Math
					.sin(gyroscopeOrientation[2]));

			// Find the gravity component of the Y-axis
			// = g*-sin(pitch);
			components[1] = (float) (SensorManager.GRAVITY_EARTH * -Math
					.sin(gyroscopeOrientation[1]));

			// Find the gravity component of the Z-axis
			// = g*cos(pitch)*cos(roll);
			components[2] = (float) (SensorManager.GRAVITY_EARTH
					* Math.cos(gyroscopeOrientation[1]) * Math
					.cos(gyroscopeOrientation[2]));

			// Subtract the gravity component of the signal
			// from the input acceleration signal to get the
			// tilt compensated output.
			linearAcceleration[0] = (this.acceleration[0] - components[0]);
			linearAcceleration[1] = (this.acceleration[1] - components[1]);
			linearAcceleration[2] = (this.acceleration[2] - components[2]);

			linearAcceleration = mfLinearAcceleration
					.filterFloat(linearAcceleration);
		}

		timestampOld = timestamp;

		notifyLinearAccelerationObserver();
	}
	*/
	

}
