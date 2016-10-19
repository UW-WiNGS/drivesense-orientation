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
		
		//GPSEvaluation.start();
		
		SlopeAwareAlignment.start();
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
