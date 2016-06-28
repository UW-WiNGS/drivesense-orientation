package main;

import java.util.ArrayList;
import java.util.List;

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
	
	
	private static String input = "/home/lkang/Dropbox/projects/drivesense/data/lei_db";
	private static String output = "/home/lkang/Dropbox/projects/drivesense/data/lei_dat";
	
	public static void start() {
		
	}
	
	
	public static void writeTraces(String path, long start) {
		
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
		
		for(Trace trace: speed) {
			Log.log(trace.toString());
		}
		

	}

}
