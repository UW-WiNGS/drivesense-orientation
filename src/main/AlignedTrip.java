package main;

import java.util.ArrayList;
import java.util.List;

import database.SqliteAccess;

import tracereplay.DirectoryWalker;
import tracereplay.PreProcess;
import tracereplay.ReadWriteTrace;
import utility.Log;
import utility.Trace;

public class AlignedTrip {
	
	private static String input = "/home/lkang/Dropbox/projects/drivesense/data/aligned/raw/";
	private static String output = "/home/lkang/Dropbox/projects/drivesense/data/aligned/dat/";

	
	public static void start() {
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
	}
	
	public static void loadTrip(String path, long start) {
		List<Trace> accelerometer = SqliteAccess.loadSensorData(path, start, Trace.ACCELEROMETER);
		List<Trace> gyroscope = SqliteAccess.loadSensorData(path, start, Trace.GYROSCOPE);
		List<Trace> rotation_matrix = SqliteAccess.loadSensorData(path, start, Trace.ROTATION_MATRIX);
		List<Trace> gps = SqliteAccess.loadSensorData(path, start, Trace.GPS);
		
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
		
		List<Trace> accumulatedGyro = new ArrayList<Trace>();
		Trace sum = new Trace(3);
		for(Trace trace: smoothed_gyroscope) {
			for(int i = 0; i < trace.dim; ++i) {
				sum.values[i] += trace.values[i];
				
			}
			Trace ntr = new Trace(3);
			ntr.copyTrace(sum);
			ntr.time = trace.time;
			accumulatedGyro.add(ntr);
		}
		ReadWriteTrace.writeFile(accumulatedGyro, output.concat("./accumulated_gyroscope.dat"));
		
	}
	
	

}
