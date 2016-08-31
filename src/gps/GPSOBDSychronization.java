package gps;

import io.ReadWriteTrace;

import java.util.List;

import utility.Constants;
import utility.Log;
import utility.PreProcess;
import utility.Trace;

public class GPSOBDSychronization {
		
	/*
	private static String dbPath = "/u/l/k/lkang/Dropbox/projects/obd/data/rawdb/";
	private static String datPath = "/u/l/k/lkang/Dropbox/projects/obd/data/rawdat/";
	private static String outputPath = "/u/l/k/lkang/Dropbox/projects/obd/data/workingset/";
	
	
	private static String name = "lei";
	private static double accumulated_distance = 0.0; //in mile
	
	*/

	
	
	/**
	 * assume that the difference between OBD speed and gps is within 10 seconds
	 * @param speed
	 * @param gps: 
	 * @param initshift: the initial time difference between gps and obd speed
	 * @return -1 on failure
	 */
	public static long sychronization(List<Trace> speed, List<Trace> gps) {
		if(gps.size() < 10) {
			return -1;
		}
		Trace first = gps.get(0);
		Trace last = gps.get(gps.size() - 1);
		if(GPSAbstraction.distance(first, last) < 100) {
			return -1;
		}
		double rate = 2.0;
		List<Trace> spds = PreProcess.interpolate(speed, rate);
		final long diff = -100000;
		long desireshift = 0;
		double minerr = Double.MAX_VALUE;
		for(long shift = diff; shift <= Math.abs(diff); shift += 250) {
			double sumabserr = 0.0;
			double sumerr = 0.0;
			for(int i = 0; i < gps.size() - 1; ++i) {
				Trace cur = gps.get(i);
				Trace next = gps.get(i + 1);
				long time = cur.time + shift;
				double speedinmps = GPSAbstraction.distance(cur, next)/((next.time - cur.time)/1000.0);
				Trace spd = PreProcess.getTraceAt(spds, time);
				double err = speedinmps;
				if(spd!=null) {
					err = spd.values[0] * Constants.kKmPHToMeterPS - speedinmps;
				}
				sumerr += err;
				sumabserr += Math.abs(err);
				
			}
			if(sumabserr < minerr) {
				minerr = sumabserr;
				desireshift = shift;
			}
		}
		return desireshift;
	}
	
}
