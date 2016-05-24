package database;

import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.sql.Statement;
import java.util.ArrayList;
import java.util.List;

import utility.Trace;

public class SqliteAccess {
	
	private static Connection connection = null;
	private final static String TAG = "SqliteAccess";
	 
	private static String prefix = "jdbc:sqlite:"; ///home/lkang/Dropbox/projects/obd/data/test_trims/raw/1395689940634.db
	private static String output = "/home/lkang/Dropbox/projects/obd/data/test_trims/3/";
	
	private static void close() {
		try {
			if(connection != null) {
				connection.close();
			}
		} catch(SQLException e) {
	        // connection close failed.
			System.err.println(e);  
		}
	}
	
	private static void connect (String dbpath) {		  
		try {
			Class.forName("org.sqlite.JDBC");
			connection = DriverManager.getConnection(dbpath);
		} catch (ClassNotFoundException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		} catch (SQLException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	

	public static List<Trace> loadSensorData(String path, String type, long start) {
		
		connect(prefix.concat(path));
		
		List<Trace> res = new ArrayList<Trace>();
		
		Statement statement = null;
		ResultSet rs = null;
		//long start = Long.MAX_VALUE;
		String table = null;
		int dim = 3;
		if(type.equals(Trace.ACCELEROMETER)) {
			table = "accelerometer";
		} else if (type.equals(Trace.GYROSCOPE)) {
			table = "gyroscope";
		} else if (type.equals(Trace.GPS)) {
			table = "gps";
		} else if (type.equals(Trace.ROTATION_MATRIX)) {
			table = "rotation_matrix";
			dim = 9;
		} else  {
			assert 0 == 1;
		}
		
		try {
			statement = connection.createStatement();
		    rs = statement.executeQuery("select * from ".concat(table));
		    while(rs.next()) {
		    	Trace trace = new Trace(dim);
		    	long time = Long.parseLong(rs.getString("time"));

		    	for(int i = 0; i < dim; ++i) {
		    		trace.values[i] = Double.parseDouble(rs.getString("x".concat(String.valueOf(i))));
		    	}
		    	trace.time = time - start;
		    	res.add(trace);
		    }
		} catch (SQLException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
	    close();
	    return res;
	}

}
