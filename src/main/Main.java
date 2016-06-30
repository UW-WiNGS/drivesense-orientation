package main;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

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
		start();
	}
	
	
	private static String input = "/home/lkang/Dropbox/projects/drivesense/data/lei_db/";
	private static String output = "/home/lkang/Dropbox/projects/drivesense/data/lei_dat/";
	
	public static void start() {
		List<String> files = DirectoryWalker.getFileNames(input);
		for(String file: files) {
			String name = file.substring(0, 13);
			String ipath = input.concat(file);
			long start = Long.valueOf(name);
			String opath = output.concat(name);
			
			DirectoryWalker.createFolder(opath);
			processTrip(ipath, start, opath);
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
	
	public static void processTrip(String path, long start, String opath) {
		List<Trace> speed = loadOBDSpeed(path, start);
		List<Trace> accelerometer = SqliteAccess.loadSensorData(path, start, Trace.ACCELEROMETER);
		List<Trace> gyroscope = SqliteAccess.loadSensorData(path, start, Trace.GYROSCOPE);
		List<Trace> rotation_matrix = SqliteAccess.loadSensorData(path, start, Trace.ROTATION_MATRIX);
		List<Trace> gps = SqliteAccess.loadSensorData(path, start, Trace.GPS);
		
		/*
		ReadWriteTrace.writeFile(speed, opath + "/speed.dat");
		ReadWriteTrace.writeFile(accelerometer, opath + "/accelerometer.dat");
		ReadWriteTrace.writeFile(gyroscope, opath + "/gyroscope.dat");
		ReadWriteTrace.writeFile(rotation_matrix, opath + "/rotation_matrix.dat");
		ReadWriteTrace.writeFile(gps, opath + "/gps.dat");
		*/
		
		/*
		List<List<Trace> > traces = new ArrayList<List<Trace> >();
		traces.add(rotation_matrix);
		traces.add(accelerometer);
		List<Trace> projected_accelerometer = traceReplay(traces);
		ReadWriteTrace.writeFile(projected_accelerometer, opath + "/projected_accelerometer.dat");
		*/
		
		List<Trace> smoothed_accelerometer = PreProcess.exponentialMovingAverage(accelerometer);
		List<Trace> smoothed_gyroscope = PreProcess.exponentialMovingAverage(gyroscope);
		
		List<Trace> window_accelerometer = new LinkedList<Trace>();
		RealTimeBehaviorDetector detector = new RealTimeBehaviorDetector();
		RotationMatrix rm = new RotationMatrix();
		final int kWindowSize = 10;
		Trace rotation = null;
		for(Trace trace: smoothed_accelerometer) {
			window_accelerometer.add(trace);
			if(window_accelerometer.size() > kWindowSize) {
				boolean steady = detector.stopped(window_accelerometer);
				if(steady && rm.rm_set == false) {
					List<Trace> sub = PreProcess.extractSubList(rotation_matrix, window_accelerometer.get(0).time, window_accelerometer.get(kWindowSize - 1).time);
					rotation = PreProcess.getAverage(sub);
					if(rotation !=null) {
						rm.rotation_matrix = rotation.values;
						rm.setRotationMatrix(rotation);
						rm.rm_set = true;
						Log.log("rotation matrix is set");
					}
				}
				window_accelerometer.remove(0);
			}
		}
		List<Trace> rotated_gyroscope = new ArrayList<Trace>();
		for(Trace trace: smoothed_gyroscope) {
			Trace tTrace = rm.Rotate(trace);
			rotated_gyroscope.add(tTrace);
		}
		
		List<Trace> rotated_accelerometer = new ArrayList<Trace>();
		for(Trace trace: smoothed_accelerometer) {
			Trace tTrace = rm.Rotate(trace);
			Trace gyro = PreProcess.getTraceAt(rotated_gyroscope, tTrace.time);
			if(gyro == null) continue;
			if(Math.abs(gyro.values[2]) > 0.05) continue;
			rotated_accelerometer.add(tTrace);
		}

		
		ReadWriteTrace.writeFile(rotated_accelerometer, opath + "/rotated_accelerometer.dat");
		ReadWriteTrace.writeFile(rotated_gyroscope, opath + "/rotated_gyroscope.dat");

		/*
		double slope = rm.curveFit(rotated_accelerometer);
		rm.setUnitVector(rotated_accelerometer, slope);
		*/
		TransformationHelper helper = new TransformationHelper();
		List<Trace> projected_accelerometer = helper.transformByAccelerometer(smoothed_accelerometer, rotation);
		List<Trace> projected_gyroscope = helper.transform(smoothed_gyroscope);
				
		
		ReadWriteTrace.writeFile(projected_accelerometer, opath + "/projected_accelerometer.dat");
		ReadWriteTrace.writeFile(projected_gyroscope, opath + "/projected_gyroscope.dat");

	}
	
	
	
	
	
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

}
