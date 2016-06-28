package main;

import java.util.ArrayList;
import java.util.List;

import tracereplay.DirectoryWalker;
import tracereplay.ReadWriteTrace;
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
			writeTraces(ipath, start, opath);
			break;
		}
	}
	
	
	public static void writeTraces(String path, long start, String opath) {
		
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
		List<Trace> accelerometer = SqliteAccess.loadSensorData(path, start, Trace.ACCELEROMETER);
		List<Trace> gyroscope = SqliteAccess.loadSensorData(path, start, Trace.GYROSCOPE);
		List<Trace> rotation_matrix = SqliteAccess.loadSensorData(path, start, Trace.ROTATION_MATRIX);
		List<Trace> gps = SqliteAccess.loadSensorData(path, start, Trace.GPS);
		
		
		ReadWriteTrace.writeFile(speed, opath + "/speed.dat");
		ReadWriteTrace.writeFile(accelerometer, opath + "/accelerometer.dat");
		ReadWriteTrace.writeFile(gyroscope, opath + "/gyroscope.dat");
		ReadWriteTrace.writeFile(rotation_matrix, opath + "/rotation_matrix.dat");
		ReadWriteTrace.writeFile(gps, opath + "/gps.dat");
	}
	
	public static void traceReplay(List<List<Trace> > input) {
		int num = input.size();
		int index[] = new int[num];
		for(int i = 0; i < num; ++i) {
			index[i] = 0;
		}
		while(true) {
			long time = -1;
			for(int i = 0; i < num; ++i) {
				
			}
		}
	}

}
