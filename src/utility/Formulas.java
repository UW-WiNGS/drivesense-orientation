package utility;

import java.util.List;

public class Formulas {

	public static double vectorLength(Trace trace) {
		return vectorLength(trace, trace.dim);
	}
	/**
	 * The name vectorSum is very very confusing!!!!!
	 * It is actually the length of a vector!
	 * @param trace
	 * @param dim
	 * @return
	 */

	public static double vectorLength(Trace trace, int dim) {
		double sum = 0.0;
		for(int i = 0; i < dim; ++i) {
			sum += Math.pow(trace.values[i], 2);
		}
		double res = Math.sqrt(sum);
		return res;
	}
	
	public static double[] vectorSum(Trace trace1, Trace trace2)
	{
		if(trace1.dim != trace2.dim)
		{
			throw new IllegalArgumentException("The dimension of two trace values must be the same");
		}
		double[] value1 = trace1.values;
		double[] value2 = trace2.values;
		double[] sum = new double[trace1.dim];
		for(int i = 0; i < trace1.dim; i++)
		{
			sum[i] = value1[i] + value2[i];
		}
		return sum;
	}
	
	/**
	 * Pure mathematical equations. Get the sum of distance squared from the points to the line.
	 * @param slope
	 * @param sum_x2
	 * @param sum_y2
	 * @param sum_xy
	 * @return
	 */
	
	public static double DistanceSquare(double slope, double sum_x2, double sum_y2, double sum_xy) {
		double res = (sum_y2 - 2 * slope * sum_xy + Math.pow(slope, 2) * sum_x2) / (Math.pow(slope, 2) + 1);
		return res;
	}

	/**
	 * Find the unit vector.
	 * @param v
	 * @return
	 */
	public static PairDouble UnitVector(PairDouble v) {
		double length = Math.sqrt(Math.pow(v.y, 2) + Math.pow(v.x, 2));
		if (length != 0)
		{
			return new PairDouble(v.x/length, v.y/length);
		}
		return new PairDouble(0, 0);
	}

	public static double DotProduct(PairDouble v, PairDouble u) {
		return v.x * u.x + v.y * u.y;
	}
	
	public static double DotProduct(Trace x, Trace y) {
		return x.values[0] * y.values[0] + x.values[1] * y.values[1];
	}


	/*0 to 360
	 * 
	 * degree difference from d2 to d1
	 * */
	public static double degreeDifference(double d1, double d2) {
		double diff = d2 - d1;
		if(diff > 180.0) {
			diff = diff - 360.0;
		} else if (diff < -180.0) {
			diff = diff + 360.0;
		} else {}

		return diff;
	}

	

	/**
	 * 
	 * 
	 * @param traces
	 * @return [deviation]
	 */
	public static double[] absoluteDeviation(List<Trace> traces) {
		int sz = traces.size();
		int d = traces.get(sz - 1).dim;
		
		double[] average = new double[d];
		double[] deviation = new double [d];
		for(int j = 0; j < d; ++j) {
			average[j] = 0.0;
			deviation[j] = 0.0;
		}
		for(Trace trace: traces) {
			for(int j = 0; j < d; ++j) {
				average[j] += trace.values[j];
			}
		}
		for(int j = 0; j < d; ++j) {
			average[j] /= sz;
		}
		for(Trace trace: traces) {
			for(int j = 0; j < d; ++j) {
				deviation[j] += Math.abs(average[j] - trace.values[j]);
			}
		}
		/*
		double [][] res = new double[2][d];		
		for(int j = 0; j < d; ++j) {
			deviation[j] /= sz;
			res[0][j] = average[j];
			res[1][j] = deviation[j];
		}
		*/
		return deviation;
	}
	/*
	 * For a given trace (preferably the raw accelerometer data, but apply to all)
	 * return the standard deviation of the traces
	 * */
	public static double[] standardDeviation(List<Trace> traces) {
		int sz = traces.size();
		int d = traces.get(sz - 1).dim;
		
		double[] average = new double[d];
		double[] res = new double [d];
		for(int j = 0; j < d; ++j) {
			average[j] = 0.0;
			res[j] = 0.0;
		}
		for(Trace trace: traces) {
			for(int j = 0; j < d; ++j) {
				average[j] += trace.values[j];
			}
		}
		for(int j = 0; j < d; ++j) {
			average[j] /= sz;
		}
		for(Trace trace: traces) {
			for(int j = 0; j < d; ++j) {
				res[j] += Math.pow((average[j] - trace.values[j]), 2.0);
			}
		}
		for(int j = 0; j < d; ++j) {
			res[j] = Math.sqrt(res[j]/sz);
		}
		
		return res;
	}
	
	
	public static double linear_correlation(double [] x, double [] y) {
		double corr = 0.0;
		int sz = x.length;
		double average_x = 0.0;
		double average_y = 0.0;
		for(int i = 0 ; i < sz; ++i) {
			average_x += x[i];
			average_y += y[i];
		}
		average_x /= sz;
		average_y /= sz;
		
		double upper = 0.0;
		double m_x = 0.0, m_y = 0.0;
		for(int i = 0 ; i < sz; ++i) {
			upper += (x[i] - average_x) * (y[i] - average_y);
			m_x += (x[i] - average_x) * (x[i] - average_x);
			m_y += (y[i] - average_y) * (y[i] - average_y);
		}
		if(m_x*m_y ==0 || m_x*m_y != m_x*m_y) corr = 1;
		else corr = upper / Math.sqrt(m_x * m_y);
		
		return corr;
	}
	
}
