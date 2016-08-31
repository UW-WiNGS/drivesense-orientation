package main;

import java.util.ArrayList;
import java.util.List;

import database.SqliteAccess;

import tracereplay.DirectoryWalker;
import tracereplay.PreProcess;
import tracereplay.ReadWriteTrace;
import utility.Constants;
import utility.Log;
import utility.Trace;

public class AlignedTrip {
	/*
	private static String input = "/home/lkang/Dropbox/projects/drivesense/data/aligned/raw/";
	
	*/
	private static String output = "/home/lkang/Dropbox/projects/drivesense/data/aligned/dat/";
	private static String input = "/home/lkang/Dropbox/projects/drivesense/data/micro_testing/groundtruth/trip_2/";
	
	public static void start() {
		
		List<Trace> gps = ReadWriteTrace.readFile(input.concat("gps.dat"), 4);
		List<Trace> speed = new ArrayList<Trace>();
		for(int i = 0; i < gps.size() - 1; ++i) {
			Trace cur = gps.get(i);
			Trace next = gps.get(i + 1);
			long diffTime = next.time - cur.time;
			double diffSpeed = next.values[3] - cur.values[3];
			double acce = diffSpeed / ((double)diffTime/1000.0);
			Trace nTrace = new Trace(5);
			nTrace.time = cur.time;
			nTrace.type = cur.type;
			for(int j = 0; j < cur.dim; ++j) {
				nTrace.values[j] = cur.values[j];
			}
			nTrace.values[4] = acce;
			speed.add(nTrace);
		}
		ReadWriteTrace.writeFile(speed, input.concat("speed.dat"));
		/*
		List<String> files = DirectoryWalker.getFileNames(input);
		for(String file: files) {
			String name = file.substring(0, 13);
			String ipath = input.concat(file);
			long start = Long.valueOf(name);
			String opath = output.concat(name);			
			DirectoryWalker.createFolder(opath);
			loadTrip(ipath, start);
			break;
		}
		*/
	}
	
	public static void loadTrip(String path, long start) {
		List<Trace> accelerometer = SqliteAccess.loadSensorData(path, start, Trace.ACCELEROMETER);
		List<Trace> gyroscope = SqliteAccess.loadSensorData(path, start, Trace.GYROSCOPE);
		List<Trace> rotation_matrix = SqliteAccess.loadSensorData(path, start, Trace.ROTATION_MATRIX);
		List<Trace> gps = SqliteAccess.loadSensorData(path, start, Trace.GPS);
		List<Trace> speed = loadGPS(path, start);
		
		Log.log(accelerometer.size());
		
		List<Trace> smoothed_accelerometer = PreProcess.exponentialMovingAverage(accelerometer);
		List<Trace> smoothed_gyroscope = PreProcess.exponentialMovingAverage(gyroscope);
		List<Trace> smoothed_rm = PreProcess.exponentialMovingAverage(rotation_matrix);
		
		ReadWriteTrace.writeFile(accelerometer, output.concat("./accelerometer.dat"));
		ReadWriteTrace.writeFile(gyroscope, output.concat("./gyroscope.dat"));
		ReadWriteTrace.writeFile(rotation_matrix, output.concat("./rotation_matrix.dat"));
		ReadWriteTrace.writeFile(gps, output.concat("./gps.dat"));
		
		
		ReadWriteTrace.writeFile(smoothed_accelerometer, output.concat("./smoothed_accelerometer.dat"));
		ReadWriteTrace.writeFile(smoothed_gyroscope, output.concat("./smoothed_gyroscope.dat"));
		ReadWriteTrace.writeFile(smoothed_rm, output.concat("./smoothed_rm.dat"));
		ReadWriteTrace.writeFile(speed, output.concat("./speed.dat"));
		
		List<Trace> accumulatedGyro = new ArrayList<Trace>();
		Trace sum = new Trace(3);
		
		int lengyro = smoothed_gyroscope.size();
		
		for(int i = 0; i < lengyro - 1; ++i) {
			Trace cur = smoothed_gyroscope.get(i);
			Trace next = smoothed_gyroscope.get(i + 1);
			for(int j = 0; j < cur.dim; ++j) {
				sum.values[j] += (cur.values[j] * (next.time - cur.time)/1000.0);
				Trace ntr = new Trace(3);
				ntr.copyTrace(sum);
				ntr.time = cur.time;
				accumulatedGyro.add(ntr);
			}
		}
		
		ReadWriteTrace.writeFile(accumulatedGyro, output.concat("./accumulated_gyroscope.dat"));
		
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
	

}
