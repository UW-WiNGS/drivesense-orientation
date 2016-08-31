package gps;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import utility.Formulas;
import utility.Pattern;
import utility.Trace;

public class RoadConditionExtraction {
	
	/**
	 * Extract flat road segments based on elevation
	 * @param gps
	 * @param dim
	 * @return List<Pattern>
	 */
	public static List<Pattern> flatRoad(List<Trace> gps, int dim) {
		List<Pattern> patterns = new ArrayList<Pattern>();
		boolean flat = false;
		Pattern pat = null;
		for(int i = 0; i < gps.size() - 1; ++i) {
			Trace cur = gps.get(i);
			Trace next = gps.get(i + 1);
			if(next.values[dim-1] == cur.values[dim-1]) {
				if(!flat) {
					flat = true;
					pat = new Pattern();
					pat.start_index = i;
					pat.start = cur.time;
				}
			} else {
				if(flat) {
					flat = false;
					if(pat != null) {
						pat.end_index = i;
						pat.end = cur.time;
						patterns.add(pat);
					}
				}
			}
		}
		if(flat) {
			pat.end_index = gps.size() - 1;
			pat.end = gps.get(gps.size() - 1).time;
			patterns.add(pat);
		}		
		return patterns;
	}
	
	/**
	 * Extract up hill road segments based on elevation
	 * @param gps
	 * @param dim
	 * @return List<Pattern>
	 */
	public static List<Pattern> monotonicIncreasing(List<Trace> gps, int dim) {
		List<Pattern> patterns = new ArrayList<Pattern>();
		boolean increasing = false;
		Pattern pat = null;
		for(int i = 0; i < gps.size() - 1; ++i) {
			Trace cur = gps.get(i);
			Trace next = gps.get(i + 1);
			if(next.values[dim-1] > cur.values[dim-1]) {
				if(!increasing) {
					increasing = true;
					pat = new Pattern();
					pat.start_index = i;
					pat.start = cur.time;
				}
			} else {
				if(increasing) {
					increasing = false;
					if(pat != null) {
						pat.end_index = i;
						pat.end = cur.time;
						patterns.add(pat);
					}
				}
			}
		}
		if(increasing) {
			pat.end_index = gps.size() - 1;
			pat.end = gps.get(gps.size() - 1).time;
			patterns.add(pat);
		}	
		return patterns;
	}
	
	
	/**
	 * Extract non decreasing segments for input trace
	 * @param gps
	 * @param dim
	 * @return List<Pattern>
	 */
	public static List<Pattern> monotonicNonDecreasing(List<Trace> gps, int dim) {
		List<Pattern> patterns = new ArrayList<Pattern>();
		boolean increasing = false;
		Pattern pat = null;
		for(int i = 0; i < gps.size() - 1; ++i) {
			Trace cur = gps.get(i);
			Trace next = gps.get(i + 1);
			if(next.values[dim-1] >= cur.values[dim-1]) {
				if(!increasing) {
					increasing = true;
					pat = new Pattern();
					pat.start_index = i;
					pat.start = cur.time;
				}
			} else {
				if(increasing) {
					increasing = false;
					if(pat != null) {
						pat.end_index = i;
						pat.end = cur.time;
						patterns.add(pat);
					}
				}
			}
		}
		if(increasing) {
			pat.end_index = gps.size() - 1;
			pat.end = gps.get(gps.size() - 1).time;
			patterns.add(pat);
		}	
		return patterns;
	}
	
	/**
	 * Extract down hill road segments based on elevation
	 * @param gps
	 * @param dim
	 * @return List<Pattern>
	 */
	public static List<Pattern> monotonicDecreasing(List<Trace> gps, int dim) {
		List<Pattern> patterns = new ArrayList<Pattern>();
		boolean decreasing = false;
		Pattern pat = null;
		for(int i = 0; i < gps.size() - 1; ++i) {
			Trace cur = gps.get(i);
			Trace next = gps.get(i + 1);
			if(next.values[dim-1] < cur.values[dim-1]) {
				if(!decreasing) {
					decreasing = true;
					pat = new Pattern();
					pat.start_index = i;
					pat.start = cur.time;
				}
			} else {
				if(decreasing) {
					decreasing = false;
					if(pat != null) {
						pat.end_index = i;
						pat.end = cur.time;
						patterns.add(pat);
					}
				}
			}
		}
		if(decreasing) {
			pat.end_index = gps.size() - 1;
			pat.end = gps.get(gps.size() - 1).time;
			patterns.add(pat);
		}	
		return patterns;
	}
	
	/**
	 * Extract flat road segments based on elevation using sliding window
	 * @param gps
	 * @return List<Pattern>
	 */
	public static List<Pattern> extractFlatRoadSegment(List<Trace> gps){
		return extractStopIntervals(gps, 5, 0.25, 3);
	}
	
	static public List<Pattern> extractStopIntervals(List<Trace> traces, int wnd, double threshold, int dim) {
		
		List<Pattern> intervals = new ArrayList<Pattern>();
		int sz = traces.size();
		if(0 == sz) return intervals;
		//Log.log(Thread.currentThread().getStackTrace()[1].getMethodName(), "the size of input traces is:" + String.valueOf(sz));
		
		LinkedList<Trace> sliding = new LinkedList<Trace>();
		
		boolean in_static = false;
	
		//int d = traces.get(sz - 1).dim;
		
		Pattern inter = null;
		for(int i = 0; i < sz; ++i) {
			Trace trace = traces.get(i);
			
			sliding.add(trace);
			int len = sliding.size();
			
			if(len==wnd) {
				//Trace p = sliding.getFirst();
	
				double [] deviation = Formulas.standardDeviation(sliding);

				//Log.error(deviation[0], deviation[1], deviation[2]);
				/*detect movement*/
				boolean moving  = false;
/*				for(int j = 0; j < d; ++j) {
					if(deviation[j] > threshold) {
						moving = true;
					}
				}*/
				if(deviation[dim-1] > threshold) {
					moving = true;
				}
				/**/
				if(!moving) {
					/*static*/
					if(false==in_static) {
						in_static = true;
						
						inter = new Pattern();
						inter.start_index = i - wnd + 1;
						inter.start = traces.get(i - wnd + 1).time;
					}
				} else {
					if(true==in_static) {
						in_static = false;
						inter.end_index = i - 1;
						inter.end = traces.get(i - 1).time;
						intervals.add(inter);
					}
				}
				/**/
				sliding.removeFirst();
			}
		}
		/*
		 * Bug found by a continues points that are static
		 * if all the points are static*/
		if(null!=inter && inter.end == - 1) {
			inter.end_index = sz - 1;
			inter.end = traces.get(sz - 1).time;
			intervals.add(inter);
		}
		
		return intervals;
	}
	
	/**
	 * Extract uphill road segments based on elevation using sliding window
	 * @param gps
	 * @return List<Pattern>
	 */
	public static List<Pattern> extractUphillRoadSegment(List<Trace> traces){
		return extractAccIntervals(traces, 5, 3);
	}
	
	public static List<Pattern> extractAccIntervals(List<Trace> traces, int wnd,int dim) {
		List<Pattern> intervals = new ArrayList<Pattern>();
		int sz = traces.size();
		LinkedList<Trace> sliding = new LinkedList<Trace>();
		Pattern inter = null;
		boolean in_static = false;
		for(int i = 0; i < sz; ++i) {
			Trace trace = traces.get(i);
			sliding.add(trace);
			int len = sliding.size();	
			if(len==wnd) {
				if (isIncreasing(sliding,dim))
				{
					if(false==in_static) {
						in_static = true;
					inter = new Pattern();
					inter.start_index = i - wnd + 1;
					inter.start = traces.get(i - wnd + 1).time;
					}
				}
				else {
					if(true==in_static) {
						in_static = false;
					inter.end_index = i - 1;
					inter.end = traces.get(i - 1).time;
					intervals.add(inter);
					}
				}
				sliding.removeFirst();			
			}
		}
		if(true==in_static) {
			inter.end_index = sz - 1;
			inter.end = traces.get(sz - 1).time;
			intervals.add(inter);
		}
		return intervals;	
	}
	
	public static List<Pattern> extractNonDecIntervals(List<Trace> traces, int wnd, int dim) {
		List<Pattern> intervals = new ArrayList<Pattern>();
		int sz = traces.size();
		LinkedList<Trace> sliding = new LinkedList<Trace>();
		Pattern inter = null;
		boolean in_static = false;
		double initvalue = 0.0;
		initvalue = traces.get(0).values[dim-1];
		for(int i = 0; i < sz; ++i) {
			initvalue = traces.get(0).values[dim -1];
			Trace trace = traces.get(i);
			sliding.add(trace);
			int len = sliding.size();	

			if(len==wnd) {
				if (isNonDecreasing(sliding, dim, initvalue))
				{
					if(false==in_static) {
						in_static = true;
					inter = new Pattern();
					inter.start_index = i - wnd + 1;
					inter.start = traces.get(i - wnd + 1).time;
					initvalue = traces.get(inter.start_index).values[dim-1];
					}
				}
				else {
					if(true==in_static) {
						in_static = false;
					inter.end_index = i - 1;
					inter.end = traces.get(i - 1).time;
					intervals.add(inter);
					initvalue = traces.get(inter.end_index).values[dim-1];
					}
				}
				sliding.removeFirst();			
			}
		}
		if(true==in_static) {
			inter.end_index = sz - 1;
			inter.end = traces.get(sz - 1).time;
			intervals.add(inter);
		}
		return intervals;	
	}
	
	public static boolean isNonDecreasing (List<Trace> traces, int dim, double initvalue){
		int sz = traces.size();
		for (int i = 0; i <= sz -2; i++)
		{
			if ((traces.get(i).values[dim-1] > traces.get(i+1).values[dim-1]))// || traces.get(i+1).values[0] == 0)
			{
				if(traces.get(i+1).values[dim-1] < (traces.get(i).values[dim-1] - initvalue)*90/100 + initvalue)
				{
					return false;
				}
			}
		}
		return true;
	}
	
	public static boolean isIncreasing (List<Trace> traces, int dim){
		int sz = traces.size();
		for (int i = 0; i <= sz -2; i++)
		{
			if ((traces.get(i).values[dim-1] >= traces.get(i+1).values[dim-1]))// || traces.get(i+1).values[0] == 0)
			{
				return false;
			}
		}
		return true;
	}
}
