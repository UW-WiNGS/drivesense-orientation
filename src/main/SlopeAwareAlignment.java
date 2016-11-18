package main;

import io.DirectoryWalker;
import io.ReadWriteTrace;
import io.SqliteAccess;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import tracereplay.RealTimeBehaviorDetector;
import tracereplay.TraceReplayEngine;
import utility.Constants;
import utility.Formulas;
import utility.Log;
import utility.PreProcess;
import utility.Trace;

public class SlopeAwareAlignment {
	
	private static String input = "/home/lkang/Dropbox/projects/drivesense/data/lei_db/";
	private static String output = "/home/lkang/Dropbox/projects/drivesense/data/lei_dat/";
	
	public static void start() {
		List<String> files = DirectoryWalker.getFileNames(input);
		for(String file: files) {
			String name = file.substring(0, 13);
			String ipath = input.concat(file);
			long start = Long.valueOf(name);
			String opath = output.concat(name);
			/*
			if(!name.endsWith("1399578241231")) {
				continue;
			}
			*/
			DirectoryWalker.createFolder(opath);
			//flatRoadDetector(ipath, start, opath);
			
			//test(ipath, start, opath);
			break;
		}
	}
	
	public static List<Trace> loadGPS(String path, long start) {		
		List<Trace> speed = new ArrayList<Trace>();
		List<Trace> gps = SqliteAccess.loadSensorData(path, start, Trace.GPS);
		
		for(int i = 0; i < gps.size() - 1; ++i) {
			Trace cur = gps.get(i);
			Trace next = gps.get(i + 1);
			long diffTime = next.time - cur.time;
			double diffSpeed = next.values[2] - cur.values[2];
			double acce = diffSpeed / ((double)diffTime/1000.0);
			Trace nTrace = new Trace(2);
			nTrace.time = cur.time;
			nTrace.type = cur.type;
			nTrace.values[0] = cur.values[2];
			nTrace.values[1] = acce;
			speed.add(nTrace);
		}
		return speed;
	}
	
	
	public void test(String path, long start, String opath) {
		int end = 2000;
		List<Trace> speed = PreProcess.interpolate(loadOBDSpeed(path, start), 1.0).subList(0, 350);
		List<Trace> accelerometer = SqliteAccess.loadSensorData(path, start, Trace.ACCELEROMETER).subList(100, end);
		List<Trace> gyroscope = SqliteAccess.loadSensorData(path, start, Trace.GYROSCOPE).subList(100, end);
		List<Trace> rotation_matrix = SqliteAccess.loadSensorData(path, start, Trace.ROTATION_MATRIX).subList(100, end);
		List<Trace> gps = SqliteAccess.loadSensorData(path, start, Trace.GPS);
		
		
		List<Trace> res = loadGPS(path, start);
		res = PreProcess.exponentialMovingAverage(res, -1);
		res = PreProcess.interpolate(res, 10.0);
		//ReadWriteTrace.writeFile(res, opath + "/test.dat");
		
		
		
		List<Trace> smoothed_accelerometer = PreProcess.exponentialMovingAverage(SqliteAccess.loadSensorData(path, start, Trace.ACCELEROMETER), -1);
		List<Trace> smoothed_gyroscope = PreProcess.exponentialMovingAverage(SqliteAccess.loadSensorData(path, start, Trace.GYROSCOPE), -1);
		List<Trace> smoothed_rm = PreProcess.exponentialMovingAverage(rotation_matrix, -1);
		
		
		ReadWriteTrace.writeFile(smoothed_accelerometer, opath + "/smoothed_accelerometer.dat");
		
		List<List<Trace>> input = new ArrayList<List<Trace>>();
		input.add(accelerometer);
		input.add(gyroscope);
		input.add(rotation_matrix);
		
		RealTimeBehaviorDetector detector = new RealTimeBehaviorDetector();
		TraceReplayEngine.traceReplay(input, detector);
				
		List<List<Trace>> trainset = detector.getTrainSet();
		Trace rm = detector.getInitRM();
		Trace hrm = detector.getHorizontalRM();
		
		
		Trace gyrodrift = detector.getGyroDrift();
		
		
		Log.log(trainset.size());
		
		List<Trace> stateortheart = new ArrayList<Trace>();
		
		
		double slope = -0.16;
		
		for(int i = 0; i < trainset.size(); ++i) {
			for(Trace trace: trainset.get(i)){
				Trace cur = Formulas.rotate(trace, rm.values);
				stateortheart.add(cur);
			}
		}

		//double [] vcoeff = Formulas.curveFit(stateortheart, 0, 1);
		//Log.log(vcoeff[0], vcoeff[1]);
		//ReadWriteTrace.writeFile(stateortheart, opath + "/stateortheart.dat");

		
		
		for(int i = 0; i < trainset.size(); ++i) {
			List<Trace> trainsample = trainset.get(i);
			//Log.log(trainsample.get(0).time/1000, trainsample.get(trainsample.size() - 1).time/1000);
			List<Trace> sample = new ArrayList<Trace>();
			for(Trace trace: trainsample) {
				Trace cur = Formulas.rotate(trace, rm.values);
				sample.add(cur);
			}

						
			List<Trace> aligned = new ArrayList<Trace>();
			for(Trace trace: sample) {
				trace = Formulas.rotate(trace, hrm.values);
				aligned.add(trace);
			}
			
			List<Trace> vtrain = new ArrayList<Trace>();
			for(Trace trace: aligned) {
				trace.values[2] -= 9.35;
				if(trace.values[2] > 9.37 || trace.values[2] < 9.33)
					vtrain.add(trace);
			}
			double [] vcoeff = Formulas.curveFit(vtrain, 1, 2);
			Log.log(vtrain.size(), vcoeff[0], vcoeff[1]);
			
			
			ReadWriteTrace.writeFile(vtrain, opath + "/sample" + i + ".dat");
			
			

			slope = Math.atan(vcoeff[1]);
			//Log.log(slope, Math.toDegrees(slope), Math.sin(slope));
			
			//Log.log(slope);
			
			final double G = 9.36;
			List<Trace> valigned = new ArrayList<Trace>();
			for(Trace trace: aligned) {
				Trace ntr = new Trace(3);
				ntr.copyTrace(trace);
				ntr.values[1] = trace.values[1] / Math.cos(slope);
				ntr.values[2] = trace.values[2] - ntr.values[1] * Math.sin(slope);
				//Log.log(trace.values[2], ntr.values[2]);
				valigned.add(ntr);
			}
			
			
		}
		
		
		
		List<Trace> projected_gyroscope = new ArrayList<Trace>();
		for(Trace trace: smoothed_gyroscope) {
			Trace initrot = Formulas.rotate(trace, rm.values);
			Trace hrot = Formulas.rotate(initrot, hrm.values);
			Trace ntr = new Trace(3);
			ntr.copyTrace(hrot);
//			ntr.values[1] = hrot.values[1] / Math.cos(slope);
//			ntr.values[2] = hrot.values[2] - ntr.values[1] * Math.sin(slope);
			//Log.log(trace.values[2], ntr.values[2]);
			projected_gyroscope.add(ntr);
		}
		ReadWriteTrace.writeFile(projected_gyroscope, opath + "/projected_gyroscope.dat");
		
		List<Trace> accumulatedGyro = new ArrayList<Trace>();
		Trace sum = new Trace(3);
		int lengyro = projected_gyroscope.size();
		for(int i = 0; i < lengyro - 1; ++i) {
			Trace cur = projected_gyroscope.get(i);
			Trace next = projected_gyroscope.get(i + 1);
			cur.values[0] += 0.015;
			for(int j = 0; j < cur.dim; ++j) {
				sum.values[j] += (cur.values[j] * (next.time - cur.time)/1000.0);
				Trace ntr = new Trace(3);
				ntr.copyTrace(sum);
				ntr.time = cur.time;
				accumulatedGyro.add(ntr);
			}
		}
		ReadWriteTrace.writeFile(accumulatedGyro, opath.concat("/accumulated_gyroscope.dat"));
		
		
		List<Trace> projected_accelerometer = new ArrayList<Trace>();
		for(Trace trace: smoothed_accelerometer) {
			Trace initrot = Formulas.rotate(trace, rm.values);
			Trace hrot = Formulas.rotate(initrot, hrm.values);
			Trace ntr = new Trace(3);
			ntr.copyTrace(hrot);
			
			
			/*
			ntr.values[1] = hrot.values[1] / Math.cos(slope);
			ntr.values[2] = hrot.values[2] - ntr.values[1] * Math.sin(slope);
			//Log.log(trace.values[2], ntr.values[2]);
			
			Trace gyro = PreProcess.getTraceAt(accumulatedGyro, trace.time);
			if(gyro != null) {
				//Log.log(trace.time, Math.sin(gyro.values[0]) * 9.4);
				ntr.values[1] -= Math.sin(gyro.values[0])*3;
			}
			*/
			projected_accelerometer.add(ntr);
		}
		ReadWriteTrace.writeFile(projected_accelerometer, opath + "/projected_accelerometer.dat");
	

	
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
	
	
	private static final double MS2S = 1.0f / 1000.0f;
	private final double[] deltaRotationVector = new double[4];
	private double timestamp = 0;
	public static final double EPSILON = 0.000000001f;
	
	public Trace onGyroscopeChanged(Trace gyro) {
	  // This timestep's delta rotation to be multiplied by the current rotation
	  // after computing it from the gyro sample data.
		if (timestamp != 0) {
			//final double dT = (gyro.time - timestamp) * MS2S;
			final double dT = 1.0;
			
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
		
		//Log.log(deltaRotationVector[2]);
		
		getRotationMatrixFromVector(tr.values, deltaRotationVector);
		return tr;
	}

	
	public void flatRoadDetector(String path, long start, String opath) {
		List<Trace> speed = PreProcess.interpolate(loadOBDSpeed(path, start), 1.0);
		List<Trace> accelerometer = SqliteAccess.loadSensorData(path, start, Trace.ACCELEROMETER);
		List<Trace> gyroscope = SqliteAccess.loadSensorData(path, start, Trace.GYROSCOPE);
		List<Trace> rotation_matrix = SqliteAccess.loadSensorData(path, start, Trace.ROTATION_MATRIX);
		List<Trace> gps = SqliteAccess.loadSensorData(path, start, Trace.GPS);
		
		
		List<Trace> smoothed_accelerometer = PreProcess.exponentialMovingAverage(accelerometer, -1);
		List<Trace> smoothed_gyroscope = PreProcess.exponentialMovingAverage(gyroscope, -1);
		List<Trace> smoothed_rm = PreProcess.exponentialMovingAverage(rotation_matrix, -1);

		ReadWriteTrace.writeFile(smoothed_accelerometer, opath + "/smoothed_accelerometer.dat");
		ReadWriteTrace.writeFile(smoothed_gyroscope, opath + "/smoothed_gyroscope.dat");

		ReadWriteTrace.writeFile(speed, opath + "/speed.dat");
		
		List<Trace> subacce = PreProcess.extractSubList(smoothed_accelerometer, 130*1000, 200*1000);
		List<Trace> subgyro = PreProcess.extractSubList(smoothed_gyroscope, 130*1000, 200*1000);
		List<Trace> subrm = PreProcess.extractSubList(rotation_matrix, 130*1000, 135*1000);
		
		
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
				trace.values[0] = Formulas.linear_correlation(window, 0, 1);
				trace.values[1] = Formulas.linear_correlation(window, 1, 2);
				trace.values[2] = Formulas.linear_correlation(window, 0, 2);		
				correlations.add(trace);
			}
		}
		return correlations;
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
