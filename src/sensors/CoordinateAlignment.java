package sensors;

import io.ReadWriteTrace;

import java.util.ArrayList;
import java.util.List;

import utility.Constants;
import utility.PreProcess;
import utility.Trace;
import utility.Trip;

public class CoordinateAlignment {
	
	private static final String input = Constants.datPath.concat("lei/urban/1397761356431/");
	
	public static void start() {
		Trip trip = ReadWriteTrace.loadTrip(input);
		offlineAlignment(trip);
	}
	
	public static List<Trace> offlineAlignment(Trip trip) {
		List<Trace> res = new ArrayList<Trace>();
		List<Trace> accelerometer = PreProcess.exponentialMovingAverage(trip.accelerometer_, -1);
		List<Trace> gyroscope = PreProcess.exponentialMovingAverage(trip.gyroscope_, -1);
		List<Trace> speed = calculateAccelerationByOBD(PreProcess.interpolate(trip.speed_, 2.0));
		
		
		
		
		
		return res;
	}
	
	private static List<Trace> calculateAccelerationByOBD(List<Trace> speed) {
		List<Trace> res = new ArrayList<Trace>();
		for(int i = 0; i < speed.size() - 1; ++i) {
			Trace cur = speed.get(i);
			Trace next = speed.get(i + 1);
			Trace ntr = new Trace(2);
			ntr.time = cur.time;
			ntr.values[0] = cur.values[0] * Constants.kKmPHToMeterPS;
			ntr.values[1] = (next.values[0] - cur.values[0]) * Constants.kKmPHToMeterPS / ((next.time - cur.time)/1000.0);
			res.add(ntr);
		}
		return res;
	}

}
