package utility;

import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import org.apache.commons.math3.fitting.PolynomialCurveFitter;
import org.apache.commons.math3.fitting.WeightedObservedPoints;


public class Formulas {
	
	

	public static Trace percentileAccuracy(List<Trace> errors, final int index) {
		
		Collections.sort(errors, new Comparator<Trace>() {
			public int compare(Trace tr0, Trace tr1) {
				if (tr0.values[index] < tr1.values[index])
					return -1;
				else if (tr0.values[index] > tr1.values[index])
					return 1;
				else
					return 0;
	        }
	    });
		int num = 10;
		int sz = errors.size();
		if(sz < num) {
			return null;
		}
		
		Trace res = new Trace(num);
		for(int i = 0; i < num; ++i) {
			res.values[i] = errors.get(sz/num * (i + 1) - 1).values[index];
		}
		return res;
	}
	
	/**
	 * Calculate the euclidean distance between two traces
	 * @param tr0
	 * @param tr1
	 * @return
	 */
	public static double euclideanDistance(Trace tr0, Trace tr1) {
		double res = 0.0;
		double sum = 0.0;
		for(int i = 0; i < tr0.values.length; ++i) {
			sum += Math.pow(tr1.values[i] - tr0.values[i], 2.0);
		}
		res = Math.sqrt(sum);
		return res;
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
	
	
	public static Trace rotate(Trace raw_tr, double[] rM) {
		Trace calculated_tr = new Trace();
		calculated_tr.time = raw_tr.time;
		double x, y, z;
		x = raw_tr.values[0];
		y = raw_tr.values[1];
		z = raw_tr.values[2];

		calculated_tr.values[0] = x * rM[0] + y * rM[1] + z * rM[2];
		calculated_tr.values[1] = x * rM[3] + y * rM[4] + z * rM[5];
		calculated_tr.values[2] = x * rM[6] + y * rM[7] + z * rM[8];

		return calculated_tr;
	}

	
	public static double linear_correlation(List<Trace> input, int x, int y) {
		double corr = 0.0;
		int sz = input.size();
		double average_x = 0.0;
		double average_y = 0.0;
		for(int i = 0 ; i < sz; ++i) {
			average_x += input.get(i).values[x];
			average_y += input.get(i).values[y];
		}
		average_x /= sz;
		average_y /= sz;
		
		double upper = 0.0;
		double m_x = 0.0, m_y = 0.0;
		for(int i = 0 ; i < sz; ++i) {
			double tmpx = input.get(i).values[x];
			double tmpy = input.get(i).values[y];
			upper += (tmpx - average_x) * (tmpy - average_y);
			m_x += (tmpx - average_x) * (tmpx - average_x);
			m_y += (tmpy - average_y) * (tmpy - average_y);
		}
		if(m_x*m_y ==0 || m_x*m_y != m_x*m_y) corr = 1;
		else corr = upper / Math.sqrt(m_x * m_y);
		
		return corr;
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
	
	public static double[] curveFit(List<Trace> acce, int i, int j) {
		final WeightedObservedPoints obs = new WeightedObservedPoints();
		for(Trace trace: acce) {
			double x = trace.values[i];
			double y = trace.values[j];
			obs.add(x, y);
		}
		// Instantiate a third-degree polynomial fitter.
		final PolynomialCurveFitter fitter = PolynomialCurveFitter.create(1);
		// Retrieve fitted parameters (coefficients of the polynomial function).
		final double[] coeff = fitter.fit(obs.toList());
		return coeff;
	}
	
	public static Trace getUnitVector(Trace input) {
		Trace res = new Trace(3);
		double sum = Math.sqrt(Math.pow(input.values[0], 2.0) + Math.pow(input.values[1], 2.0));
		res.setValues(input.values[0]/sum, input.values[1]/sum, 0.0);
		return res;
	}

	public static Trace rotationMatrixBetweenHorizontalVectors(Trace v0, Trace v1) {
		Trace res = new Trace(9);
		double cos_theta = v0.values[0] * v1.values[0] + v0.values[1] * v1.values[1];
		double sin_theta = v0.values[0] * v1.values[1] - v0.values[1] * v1.values[0];
		res.values[0] = cos_theta;
		res.values[1] = - sin_theta;
		res.values[2] = 0.0;
		res.values[3] = sin_theta;
		res.values[4] = cos_theta;
		for(int i = 5; i < 8; ++i) {
			res.values[i] = 0.0;
		}
		res.values[8] = 1.0;
		return res;
	}
	
	/**
	 * TODO --- test
	 * @param v0
	 * @param v1
	 * @return
	 */
	public static Trace rotationMatrixBetweenVerticalVectors(Trace v0, Trace v1) {
		Trace res = new Trace(9);
		double cos_theta = v0.values[1] * v1.values[1] + v0.values[2] * v1.values[2];
		double sin_theta = v0.values[1] * v1.values[2] - v0.values[2] * v1.values[1];
		for(int i = 0; i < 9; ++i) {
			res.values[i] = 0.0;
		}
		res.values[0] = 1.0;
		res.values[4] = cos_theta;
		res.values[5] = - sin_theta;
		res.values[7] = sin_theta;
		res.values[8] = cos_theta;
		return res;
	}
}
