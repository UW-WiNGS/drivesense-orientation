package main;

import io.DirectoryWalker;
import io.ReadWriteTrace;
import io.SqliteAccess;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import utility.Constants;
import utility.Log;
import utility.Trace;
import utility.Trip;

public class Main {

	/**
	 * @param args
	 */	
	public static void main(String[] args) {
		//System.out.println("Hello DriveSense-Orientation!");
		
		GPSEvaluation.start();
	}
	
	
	
	public static void loadSensorData() {
		List<String> rawdat = DirectoryWalker.recursive(Constants.datPath);
		List<String> rawdb = DirectoryWalker.recursive(Constants.dbPath);
		
		Map<String, String> path = new HashMap<String, String>();
		for(String file: rawdat) {
			int sz = file.length();
			String[] parts = file.split("/");
			int len = parts.length;
			String name = parts[len - 2].concat(".db");
			String filename = parts[len - 1];
			int index = sz - filename.length();
			String prefix = file.substring(0, index);
			if(!path.containsKey(name)) {
				path.put(name, prefix);
			}
		}
		
		for(String file: rawdb) {
			Log.log(file);
			String[] parts = file.split("/");
			int len = parts.length;
			String name = parts[len - 1];
			long start = 0;
			try {
				start = Long.parseLong(parts[len - 1].substring(0, 13));
			} catch (Exception e) {
				continue;
			}
			if(!path.containsKey(name)) continue;
			List<Trace> accelerometer = SqliteAccess.loadSensorData(file, start, Trace.ACCELEROMETER);
			ReadWriteTrace.writeFile(accelerometer, path.get(name).concat("accelerometer.dat"));
			List<Trace> gyroscope = SqliteAccess.loadSensorData(file, start, Trace.GYROSCOPE);
			ReadWriteTrace.writeFile(gyroscope, path.get(name).concat("gyroscope.dat"));
			List<Trace> rotation_matrix = SqliteAccess.loadSensorData(file, start, Trace.ROTATION_MATRIX);
			ReadWriteTrace.writeFile(rotation_matrix, path.get(name).concat("rotation_matrix.dat"));
		}
	}
	
	public static void realTimeTest() {
		List<String> rawdb = DirectoryWalker.recursive(Constants.dbPath);
		for(String file: rawdb) {
			//Log.log(file);
			String[] parts = file.split("/");
			int len = parts.length;
			long start = Long.parseLong(parts[len - 1].substring(0, 13));
			Log.log(start, file);
			List<Trace> accelerometer = SqliteAccess.loadSensorData(file, start, Trace.ACCELEROMETER);
			Log.log(accelerometer.size());
			List<List<Trace>> input = new ArrayList<List<Trace>>();
			input.add(accelerometer);
			RealTimeTiltCalculation detector = new RealTimeTiltCalculation();
			traceReplay(input, detector);
			break;
		}
	}
		
	public static void traceReplay(List<List<Trace> > input, RealTimeTiltCalculation detector) {
		int num = input.size();
		int index[] = new int[num];
		for(int i = 0; i < num; ++i) {
			index[i] = 0;
		}
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
	}
}
