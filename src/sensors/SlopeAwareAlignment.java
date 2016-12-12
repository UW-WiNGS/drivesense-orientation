package sensors;

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
import utility.Pattern;
import utility.PreProcess;
import utility.Trace;
import utility.Trip;

public class SlopeAwareAlignment {
	
	
	private static String TAG = "SlopeAwareAlignment";
	
	public static void start() {
		//test();
		
		//iterate();
		
		trainlengthAndAccuracy();
	}
	
	public static void iterate() {
		List<String> folders = DirectoryWalker.getFolders(Constants.datPath);
		
		int counter = 0;
		for(String directory: folders) {
			List<Trip> trips = ReadWriteTrace.loadTrips(directory.concat("/urban"));
			//List<Trip> highway = ReadWriteTrace.loadTrips(directory.concat("/highway"));
			//trips.addAll(highway);
			List<Trace> output = new ArrayList<Trace>();	
			for(Trip trip: trips) {				
				//boolean changed = testTrip(trip);
				//Log.log(TAG, trip.path, changed);
				
				/*
				List<Trace> stops = stopDetection(trip);
				output.addAll(stops);
				if(output.size() >= 1000) {
					break;
				}
				*/
			}
			//ReadWriteTrace.writeFile(output, Constants.kAlterSenseOutput.concat("stopextraction/" + String.valueOf(counter)));
			counter++;
		}
	}
	
	
	private static void trainlengthAndAccuracy() {
		List<String> folders = DirectoryWalker.getFolders(Constants.datPath);
		List<Trip> trips = new ArrayList<Trip>();
		for(String directory: folders) {
			trips.addAll(ReadWriteTrace.loadTrips(directory.concat("/urban")));
			if(trips.size() >= 100) {
				break;
			}
		}
		
		List<Trace> output = new ArrayList<Trace>();
		for(int i = 10; i <= 100; i+=5) {
			List<Trace> errors = alignmentError(trips, i);	
			Trace percentile = CoordinateAlignment.alignmentPencentileAccuracy(errors);
			if(percentile == null) continue;
			output.add(percentile);
		}
		ReadWriteTrace.writeFile(output, Constants.kAlterSenseOutput.concat("trainlength/error.dat"));
	}
	private static List<Trace> alignmentError(List<Trip> trips, int len) {
		RealTimeBehaviorDetector detector = new RealTimeBehaviorDetector();
		detector.setTrainLength(len);
		List<Trace> errors = new ArrayList<Trace>();
		for(Trip trip: trips) {
			List<Trace> accelerometer = PreProcess.exponentialMovingAverage(trip.accelerometer_, -1);
			boolean changed = OrientationChangeDetection.orientationChanged(accelerometer);
			if(changed) continue;
			CoordinateAlignment.trainDetector(detector, trip);
			List<Trace> speed = CoordinateAlignment.calculateAccelerationByOBD(PreProcess.interpolate(trip.speed_, 2.0));		

			List<Trace> projected_accelerometer = CoordinateAlignment.alignAccelerometer(detector, trip);
			List<Trace> tmperror = CoordinateAlignment.alignmentError(speed, projected_accelerometer);
			if(tmperror == null) {
				continue;
			}
			errors.addAll(tmperror);
			
			if(errors.size() >= 10000) break;
		}
		return errors;
	}
	
	
	private static List<Trace> stopDetection(Trip trip) {
		List<Trace> accelerometer = PreProcess.exponentialMovingAverage(trip.accelerometer_, -1);
		List<Trace> speed = CoordinateAlignment.calculateAccelerationByOBD(PreProcess.interpolate(trip.speed_, 2.0));
		
		List<Trace> res = new ArrayList<Trace>();
		List<Pattern> patterns = new ArrayList<Pattern>();
		
		boolean inpattern = false;
		Pattern pattern = null;
		for(int i = 0; i < speed.size(); ++i) {
			Trace curspeed = speed.get(i);
			if(curspeed.values[0] == 0.0) {
				if(inpattern == false) {
					pattern = new Pattern();
					pattern.start = curspeed.time;
					inpattern = true;
				}
			} else {
				if(inpattern == true) {
					pattern.end = curspeed.time;
					List<Trace> substop = PreProcess.extractSubList(accelerometer, pattern.start + 1000, pattern.end - 1000);
					
					if(substop.size() > 15) {
						List<Trace> variance = SensorCluster.slidingVariance(substop, 15);
						res.addAll(variance);
					}
					pattern = null;
					inpattern = false;
				}
			}
		}
		return res;
	}
	

	
	private static void trainSetTest(RealTimeBehaviorDetector detector, String output) {
		
		List<List<Trace>> trainset = detector.getTrainSet();
		int sz = trainset.size();
		Trace rm = detector.getInitRM();
		Trace hrm = detector.getHorizontalRM();
		
		for(int i = 0; i < trainset.size(); ++i) {
			List<Trace> subset = trainset.get(i);
			List<Trace> projected = new ArrayList<Trace>();
			Log.log(TAG, subset.size());
			for(Trace trace: subset) {
				Trace initrot = Formulas.rotate(trace, rm.values);
				Trace hrot = Formulas.rotate(initrot, hrm.values);
				
				Trace ntr = new Trace(3);
				ntr.copyTrace(hrot);
				projected.add(ntr);
			}
			ReadWriteTrace.writeFile(projected, output.concat("trainingset/" + String.valueOf(i)));
			if(i > 10) {
				break;
			}
		}
		
	}
	
	private static void test() {
		String inputfile = Constants.datPath.concat("lei/urban/1397761356431/");
		String output = Constants.kSlopeSenseOutput.concat("test/");
		Trip trip = ReadWriteTrace.loadTrip(inputfile);
		//boolean changed = testTrip(trip);
		//Log.log(TAG, trip.path, changed);

		
		List<Trace> accelerometer = PreProcess.exponentialMovingAverage(trip.accelerometer_, -1);
		List<Trace> gyroscope = PreProcess.exponentialMovingAverage(trip.gyroscope_, -1);
		List<Trace> rotation_matrix = PreProcess.exponentialMovingAverage(trip.rotation_matrix_, -1);
		List<Trace> speed = CoordinateAlignment.calculateAccelerationByOBD(PreProcess.interpolate(trip.speed_, 2.0));

		//train detector
		List<List<Trace>> input = new ArrayList<List<Trace>>();
		input.add(accelerometer);
		input.add(gyroscope);
		input.add(rotation_matrix);
		RealTimeBehaviorDetector detector = new RealTimeBehaviorDetector();
		TraceReplayEngine.traceReplay(input, detector);
		
		
		trainSetTest(detector, output);
		
		Trace rm = detector.getInitRM();
		Trace hrm = detector.getHorizontalRM();
		if(rm == null || hrm == null) {
			Log.log(TAG, "train failed");
		}
		int reverse_counter = 0;
		List<Trace> projected_accelerometer = new ArrayList<Trace>();
		for(Trace trace: accelerometer) {
			Trace initrot = Formulas.rotate(trace, rm.values);
			Trace hrot = Formulas.rotate(initrot, hrm.values);
			
			Trace ntr = new Trace(3);
			ntr.copyTrace(hrot);
			projected_accelerometer.add(ntr);
			
			long time = ntr.time;
			Trace curspeed = PreProcess.getTraceAt(speed, time);
			if(curspeed != null) {
				double correct = ntr.values[1] - curspeed.values[1];
				double reverse = ntr.values[1] + curspeed.values[1];
				if(Math.abs(reverse) < Math.abs(correct)) {
					reverse_counter ++;
				}
			}
		}
		if(reverse_counter >= 0.3 * projected_accelerometer.size()) {
			for(Trace trace: projected_accelerometer) {
				trace.values[0] = -trace.values[0];
				trace.values[1] = -trace.values[1];
			}
		}
		
		
		ReadWriteTrace.writeFile(accelerometer, output.concat("accelerometer.dat"));
		ReadWriteTrace.writeFile(gyroscope, output.concat("gyroscope.dat"));
		ReadWriteTrace.writeFile(projected_accelerometer, output.concat("projected_accelerometer.dat"));
		ReadWriteTrace.writeFile(speed, output.concat("speed.dat"));
		
	}
	
	private static boolean testTrip(Trip trip) {
		List<Trace> speed = PreProcess.interpolate(trip.speed_, 2.0);
		List<Trace> accelerometer = PreProcess.exponentialMovingAverage(trip.accelerometer_, -1);
		List<Trace> gyroscope = PreProcess.exponentialMovingAverage(trip.gyroscope_, -1);
		List<Trace> rotation_matrix = PreProcess.exponentialMovingAverage(trip.rotation_matrix_, -1);

		boolean changed = OrientationChangeDetection.orientationChanged(accelerometer);
		
		return changed;
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
