/*
	Simbicon 1.5 Controller Editor Framework, 
	Copyright 2009 Stelian Coros, Philippe Beaudoin and Michiel van de Panne.
	All rights reserved. Web: www.cs.ubc.ca/~van/simbicon_cef

	This file is part of the Simbicon 1.5 Controller Editor Framework.

	Simbicon 1.5 Controller Editor Framework is free software: you can 
	redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	Simbicon 1.5 Controller Editor Framework is distributed in the hope 
	that it will be useful, but WITHOUT ANY WARRANTY; without even the 
	implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
	See the GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with Simbicon 1.5 Controller Editor Framework. 
	If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <limits>

#include <Utils/Utils.h>
#include <MathLib/Vector3d.h>
#include <MathLib/MathLib.h>

//#define FANCY_SPLINES

/**
	This class is used to represent generic trajectories. The class parameter T can be anything that provides basic operation such as addition and subtraction.
	We'll define a trajectory that can be parameterized by a one-d parameter (called t). Based on a set of knots ( tuples <t, T>), we can evaluate the 
	trajectory at any t, through interpolation. This is not used for extrapolation. Outside the range of the knots, the closest known value is returned instead.
*/

template <class T> class GenericTrajectory{
private:
	DynamicArray<double> tValues;
	DynamicArray<T> values;

	// A caching variable to optimize searching
	volatile int lastIndex;


	/**
		This method returns the index of the first knot whose value is larger than the parameter value t. If no such index exists (t is larger than any
		of the values stored), then values.size() is returned.
	*/
	int getFirstLargerIndex(double t){
        int size = static_cast<int>(tValues.size());
		if( size == 0 ) 
			return 0;
		if( t < tValues[(lastIndex+size-1)%size] )
			lastIndex = 0;
		for (int i = 0; i<size;i++){
			int index = (i + lastIndex) % size;
			if (t < tValues[index]) {
				lastIndex = index;
				return index;
			}
		}
		return size;
	}

public:
	GenericTrajectory(void){
		lastIndex = 0;
	}
	GenericTrajectory( GenericTrajectory<T>& other ){
		lastIndex = 0;
		copy( other );
	}
	~GenericTrajectory(void){
		clear();
	}

	/**
		This method performs linear interpolation to evaluate the trajectory at the point t
	*/
	T evaluate_linear(double t){
		int size = tValues.size();
		if (t<=tValues[0]) return values[0];
		if (t>=tValues[size-1])	return values[size-1];
		int index = getFirstLargerIndex(t);
		
		//now linearly interpolate between inedx-1 and index
		t = (t-tValues[index-1]) / (tValues[index]-tValues[index-1]);
		return (values[index-1]) * (1-t) + (values[index]) * t;
	}


	/**
		This method interprets the trajectory as a Catmul-Rom spline, and evaluates it at the point t
	*/
	T evaluate_catmull_rom(double t){
		int size = tValues.size();
		if (t<=tValues[0]) return values[0];
		if (t>=tValues[size-1])	return values[size-1];
		int index = getFirstLargerIndex(t);
		
		//now that we found the interval, get a value that indicates how far we are along it
		t = (t-tValues[index-1]) / (tValues[index]-tValues[index-1]);

		//approximate the derivatives at the two ends
		double t0, t1, t2, t3;
		T p0, p1, p2, p3;
		p0 = (index-2<0)?(values[index-1]):(values[index-2]);
		p1 = values[index-1];
		p2 = values[index];
		p3 = (index+1>=size)?(values[index]):(values[index+1]);

		t0 = (index-2<0)?(tValues[index-1]):(tValues[index-2]);
		t1 = tValues[index-1];
		t2 = tValues[index];
		t3 = (index+1>=size)?(tValues[index]):(tValues[index+1]);

		double d1 = (t2-t0);
		double d2 = (t3-t1);

		if (d1 > -TINY && d1  < 0) d1 = -TINY;
		if (d1 < TINY && d1  >= 0) d1 = TINY;
		if (d2 > -TINY && d2  < 0) d2 = -TINY;
		if (d2 < TINY && d2  >= 0) d2 = TINY;

#ifdef FANCY_SPLINES
		T m1 = (p2 - p0) * (1-(t1-t0)/d1);
		T m2 = (p3 - p1) * (1-(t3-t2)/d2);
#else
		T m1 = (p2 - p0)*0.5;
		T m2 = (p3 - p1)*0.5;
#endif

		t2 = t*t;
		t3 = t2*t;

		//and now perform the interpolation using the four hermite basis functions from wikipedia
		return p1*(2*t3-3*t2+1) + m1*(t3-2*t2+t) + p2*(-2*t3+3*t2) + m2 * (t3 - t2);
	}

	/**
		Returns the value of the ith knot. It is assumed that i is within the correct range.
	*/
	T getKnotValue(int i){
		return values[i];
	}

	/**
		Returns the position of the ith knot. It is assumed that i is within the correct range.
	*/
	double getKnotPosition(int i){
		return tValues[i];
	}

	/**
		Sets the value of the ith knot to val. It is assumed that i is within the correct range.
	*/
	void setKnotValue(int i, const T& val){
		values[i] = val;
	}

	/**
		Sets the position of the ith knot to pos. It is assumed that i is within the correct range.
	*/
	void setKnotPosition(int i, double pos){
		if( i-1 >= 0               && tValues[i-1] >= pos ) return;
        if( static_cast<uint>(i+1) < tValues.size()-1 && tValues[i+1] <= pos ) return;
		tValues[i] = pos;
	}

	/**
		Return the smallest tValue or infinity if none
	*/
	double getMinPosition(){
		if( tValues.empty() ) 
			return std::numeric_limits<double>::infinity();
		return tValues.front();
	}

	/**
		Return the largest tValue or -infinity if none
	*/
	double getMaxPosition(){
		if( tValues.empty() ) 
			return -std::numeric_limits<double>::infinity();
		return tValues.back();
	}


	/**
		returns the number of knots in this trajectory
	*/
	int getKnotCount(){
                return (int)tValues.size();
	}

	/**
		This method is used to insert a new knot in the current trajectory
	*/
	void addKnot(double t, T val){
		//first we need to know where to insert it, based on the t-values
		int index = getFirstLargerIndex(t);

                if (tValues.empty()){
                    tValues.push_back(t);
                    values.push_back(val);
                }else{
                    tValues.insert(tValues.begin()+index, t);
                    values.insert(values.begin()+index, val);
                }
	}

	/**
		This method is used to remove a knot from the current trajectory.
		It is assumed that i is within the correct range.
	*/
	void removeKnot(int i){
		tValues.erase(tValues.begin()+i);
		values.erase(values.begin()+i);
	}

	/**
		This method removes everything from the trajectory.
	*/
	void clear(){
		tValues.clear();
		values.clear();
	}

	/**
		Simplify the curve by iteratively adding knots
	*/
	void simplify_catmull_rom( double maxError, int nbSamples = 100 ){

		if( getKnotCount() < 3 )
			return;

		double startTime = tValues.front();
		double endTime = tValues.back();

		GenericTrajectory<T> result;
		result.addKnot( startTime, values.front() );
		result.addKnot( endTime, values.back() );
		

		while( true ) {
			double currError = 0;
			double currErrorTime = -std::numeric_limits<double>::infinity();

			for( int i=0; i < nbSamples; ++i ) {
				double interp = (double)i / (nbSamples - 1.0);
				double time = startTime * (1 - interp) + endTime * interp;
				double error = abs( result.evaluate_catmull_rom(time) - evaluate_catmull_rom(time) );
				if( error > currError ) {
					currError = error;
					currErrorTime = time;
				}
			}
		
			if( currError <= maxError )
				break;

			result.addKnot( currErrorTime, evaluate_catmull_rom(currErrorTime) );
		}

		copy( result );
                result.clear();
	}


	void copy( GenericTrajectory<T>& other ) {

		tValues.clear();
		values.clear();
		int size = other.getKnotCount();

                if (other.tValues.size()!=other.values.size()){
                    exit(259);
                }
		for( int i=0; i < size; ++i ) {
			tValues.push_back( other.tValues[i] );
			values.push_back( other.values[i] );
		}
	}

};


typedef GenericTrajectory<double> Trajectory1D;
typedef GenericTrajectory<Vector3d> Trajectory3D;

