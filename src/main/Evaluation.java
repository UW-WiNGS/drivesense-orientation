package main;

import io.DirectoryWalker;
import io.ReadWriteTrace;
import io.SqliteAccess;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import tracereplay.RealTimeBehaviorDetector;
import utility.Log;
import utility.Trace;

public class Evaluation {
	
	private static String root = "/home/lkang/backups/projects/obd/data/rawdb/";
	private static String input = "/home/lkang/backups/projects/obd/data/rawdb/lei/1/";
	private static String output = "/home/lkang/Dropbox/projects/drivesense/data/evaluation/";

	public static void start() {
		//List<String> files = DirectoryWalker.getFileNames(input);
		
		List<String> files = DirectoryWalker.recursive(root);
		
		List<Trace> traces = new ArrayList<Trace>();
		for(String file: files) {
			int sz = file.length();
			String name = file.substring(sz - 16, sz - 3);
			
			//Log.log(name);
			long start = 0;
			try {
				start = Long.valueOf(name);
			} catch (Exception e) {
				continue;
			}
			String opath = output.concat(name);
			
			File curfile = new File(file);
			if(curfile.length() > 2e6) {
				continue;
			}
			//Log.log(start, curfile.length());
			//DirectoryWalker.createFolder(opath);
			//flatRoadDetector(ipath, start, opath);
			
			Trace trace = test(file, start, opath);
			if(null == trace) continue;
			traces.add(trace);
			
			//break;
		}
		ReadWriteTrace.writeFile(traces, output.concat("/duration.dat"));
	}
	
	

	
	public static Trace test(String path, long start, String opath) {
		//List<Trace> speed = PreProcess.interpolate(loadOBDSpeed(path, start), 1.0).subList(0, 350);
		List<Trace> accelerometer = SqliteAccess.loadSensorData(path, start, Trace.ACCELEROMETER);
		List<Trace> gyroscope = SqliteAccess.loadSensorData(path, start, Trace.GYROSCOPE);
		List<Trace> rotation_matrix = SqliteAccess.loadSensorData(path, start, Trace.ROTATION_MATRIX);
		List<Trace> gps = SqliteAccess.loadSensorData(path, start, Trace.GPS);
		
		
		
		
		List<List<Trace>> input = new ArrayList<List<Trace>>();
		input.add(accelerometer);
		input.add(gyroscope);
		input.add(rotation_matrix);
		
		RealTimeBehaviorDetector detector = new RealTimeBehaviorDetector();
		traceReplay(input, detector);
				
		List<List<Trace>> trainset = detector.getTrainSet();
		int sz = trainset.size();
		int count = 0;
		
		
		Trace trace = new Trace(3);
		if(trainset.size() == 0) return null;
		long end = trainset.get(sz - 1).get(0).time;
		trace.setValues(end, end, end);
		boolean firstset = false;
		for(List<Trace> segment: trainset) {
			int len = segment.size();
			count += len;
			if(count >= 50 && firstset == false) {
				trace.values[0] = segment.get(len - 1).time/1000;
				firstset = true;
			}
			if(count >= 150) {
				trace.values[1] = segment.get(len - 1).time/1000;
				break;
			}
		}
		if(trace.values[0] >= 600) return null;
		return trace;
	
	}
	public static void traceReplay(List<List<Trace> > input, RealTimeBehaviorDetector detector) {
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
