package tracereplay;

import java.security.Policy.Parameters;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import utility.Constants;
import utility.Log;
import utility.Trace;

public class PreProcess {
	
	
	
	/**
	 * 
	 * @param raw the input data to be interpolated 
	 * @param rate samples per second
	 * @return
	 */
	public static List<Trace> interpolate(List<Trace> raw, double rate) {
		List<Trace> res = new ArrayList<Trace>();
		
		int sz = raw.size();
		if(0==sz) return res;
		assert sz > 0 && rate>=1;
		long x = raw.get(0).time/1000 * 1000 + 1000;
		for(int i = 0; i < sz - 1; ++i) {
			Trace cur = raw.get(i);
			Trace next = raw.get(i + 1);
			if(x >= cur.time && x < next.time) {
				Trace inter = new Trace();
				inter.copyTrace(cur);
				inter.time = x;
				//Log.log(x/1000 - 1379879638, cur.values[2]);
				//assert (x/1000 - 1379879638) < 190;
				for(int j = 0; j < inter.dim; ++j) {
					long x1 = cur.time, x2 = next.time;
					double y1 = cur.values[j], y2 = next.values[j];
					
					double v1 = y1 + (x - x1) * (y2 - y1) / (x2 - x1);
					//double v2 = y2 - (x2 - x) * (y2 - y1) / (x2 - x1);
					inter.values[j] = v1;
				}
				res.add(inter);
				x += (1000.0/rate);
				--i;
			}
		}
		return res;
		
	}
	

	/*
	 * Link: http://en.wikipedia.org/wiki/Moving_average
	 * */

	/*
	 * get the average of given input
	 * */
	public static Trace getAverage(List<Trace> input) {
		return simpleMovingAverage(input, input.size()).get(0);
	}
	
	/*
	 * given a window size, get the moving average of the trace values
	 * e.g., for an array {1, 2, 3, 4, 5}, and wnd = 3
	 * the moving result is {(1+2+3)/3, (2+3+4)/3, (3+4+5)/3}
	 * return the overall average if wnd = input.size();
	 * */
	public static List<Trace> simpleMovingAverage(List<Trace> input, int wnd) 
	{
		List<Trace> res = new ArrayList<Trace>();
		int sz = input.size();
		int d = input.get(sz - 1).dim;
		double [] sum = new double[d];
		for(int j = 0; j < d; ++j) sum[j] = 0.0;
		
		for(int i = 0, len = 1; i < sz; ++i, ++len) {
			Trace temp = input.get(i);
			for(int j = 0; j < d; ++j) {
				sum[j] += temp.values[j];
			}
			/**/
			
			if(len == wnd) {
				--len;
				Trace trace = new Trace(d);
				trace.time = temp.time;
				for(int j = 0; j < d; ++j) {
					trace.values[j] = sum[j]/wnd;
					sum[j] -= input.get(i - wnd + 1).values[j];
				}
				res.add(trace);
			}

		}
		return res;
	}
	
	
	/*get the weighted moving average
	 *  e.g., for an array {1, 2, 3, 4, 5}, and wnd = 3
	 *  accu = wnd * (wnd + 1)/2 = 6
	 * the moving result is {(1*1+2*2+3*3)/accu, (2*1+3*2+4*3)/accu, (3*1+4*2+5*3)/accu}
	 * the length of result List<Trace> is wnd shorter than input traces
	 * */
	public static List<Trace> weightedMovingAverage(List<Trace> traces, int wnd) {

		int sz = traces.size();
		Log.log(Thread.currentThread().getStackTrace()[1].getMethodName(), "the size of input traces is:" + String.valueOf(sz));
		LinkedList<Trace> sliding = new LinkedList<Trace>();
		
		List<Trace> res = new ArrayList<Trace>();

		for(int i = 0; i < sz; ++i) {
			Trace trace = traces.get(i);			
			sliding.add(trace);
			int len = sliding.size();
			
			if(len==wnd) {
				double [] average = weightedAverage(sliding, wnd);
				sliding.removeFirst();
				Trace temp = new Trace();
				temp.copyTrace(trace);
				temp.values = average.clone();
				res.add(temp);
			}
		}
		return res;
		
	}
	
	static public double[] weightedAverage(List<Trace> traces, int wnd) {
		int sz = traces.size();
		int d = traces.get(sz - 1).dim;
		double accu = (double)wnd * (double)(wnd + 1) / 2.0;
			
		double[] res = new double [d];
		for(int j = 0; j < d; ++j) {
			res[j] = 0.0;
		}
		int i = 1;
		for(Trace trace: traces) {
			for(int j = 0; j < d; ++j) {
				res[j] += trace.values[j] * i;
			}
			++i;
		}
		for(int j = 0; j < d; ++j) {
			res[j] /= accu;
		}
		
		return res;
	}
	
	/*alpha is from 0 to 1
	 * if alpha is 1, the result List is exactly the same to input traces
	 * if alpha is 0, the result List is a List of the first value of input traces
	 * */
	public static List<Trace> exponentialMovingAverage(List<Trace> traces) {
		
		double alpha = Constants.kExponentialMovingAverageAlpha;
		
		List<Trace> res = new ArrayList<Trace>();
		int sz = traces.size();
		if(0==sz)
			return res;
		int d = traces.get(sz - 1).dim;
		double[] history = new double [d];
	
		for(int i = 0; i < sz; ++i) {
			Trace trace = new Trace();
			trace.copyTrace(traces.get(i));
			if(i==0) {
				history = trace.values.clone();
				res.add(trace); 
				continue;
			}
			for(int j = 0; j < d; ++j) {
				trace.values[j] = alpha * traces.get(i).values[j] + (1.0 - alpha) * history[j];
				history[j] = trace.values[j];
			}
			res.add(trace);
		}
		return res;
	}
	

	/*
	 * make the trace starts from time 0
	 * 
	 * */
	
	public static List<Trace> ClearTimeOffset(List<Trace> traces) {
		int offset = (int) traces.get(0).time;
		List<Trace> res = new ArrayList<Trace>();
		for (int i = 0; i < traces.size(); i++) {
			Trace tr = new Trace();
			tr.time = traces.get(i).time + offset;
			tr.values = traces.get(i).values;
			res.add(tr);
		}
		return res;
	}
	
	
	
	/*extract a sublist of a given List<Trace>, using binary search 
	 * 
	 * any interval in the implementation is [si, ei), from si, inclusive, to ei, exclusive.
	 * */
	
	public static int binarySearch(List<Trace> raw, int si, int ei, long target) {
		int mid = 0;
		ei -= 1;
		while(si <= ei) {
			mid = si + (ei - si)/2;
			if (raw.get(mid).time == target) {
				break;
			} else if (raw.get(mid).time < target) {
				si = mid + 1;
			} else {
				ei = mid - 1;
			}
		}
		return mid;
	}
	
	public static List<Trace> extractSubList(List <Trace> raw, long start, long end) {
		
		int si = binarySearch(raw, 0, raw.size(), start);
		//Log.error(start, raw.get(si).time);
		int ei = binarySearch(raw, si, raw.size(), end);
		//Log.error(end, raw.get(ei).time);
		return raw.subList(si, ei + 1);
	}
	
	/**
	 * 
	 * @param traces
	 * @param time
	 * @return
	 */
	public static Trace getTraceAt(List<Trace> traces, long time) {
		if(time - time/1000 * 1000 > 500) {
			time+=1000;
		}
		int sz = traces.size();
		if(sz == 0 || ! (time >= traces.get(0).time && time <= traces.get(sz - 1).time)) {
			return null;
		}
		if(sz > 1) {
			long td = traces.get(sz - 1).time - traces.get(0).time;
			int index;
			if(td/1000 + 1 == sz) {
				//sample rate is 1.0
				index = (int)(time - traces.get(0).time)/1000;
			} else {
				index = binarySearch(traces, 0, sz, time);
			}
			return traces.get(index);
		} else {
			return traces.get(0);
		}
	}
	
	
}
