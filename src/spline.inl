// Given a time between 0 and 1, evaluates a cubic polynomial with
// the given endpoint and tangent values at the beginning (0) and
// end (1) of the interval.  Optionally, one can request a derivative
// of the spline (0=no derivative, 1=first derivative, 2=2nd derivative).
template <class T>
inline T Spline<T>::cubicSplineUnitInterval(
      const T& position0,
      const T& position1,
      const T& tangent0,
      const T& tangent1,
      double normalizedTime,
      int derivative )
{
   // TODO IMPLEMENT ME (TASK 1A)
   double h00, h01, h10, h11;
   double t = normalizedTime;
   double t2 = t*t;
   double t3 = t2*t;

   if (derivative != 1 && derivative != 2) {
      if (derivative != 0) {
         cout << "Bad derivative request!" << endl;
      }
      h00 = 2*t3 - 3*t2 + 1;
      h10 = t3 - 2*t2 + t;
      h01 = -2*t3 + 3*t2;
      h11 = t3 - t2;
   } else if (derivative == 1) {
      h00 = 6*t2 - 6*t;
      h10 = 3*t2 - 4*t + 1;
      h01 = -6*t2 + 6*t;
      h11 = 3*t2 - 2*t;
   } else {
      h00 = 12*t - 6;
      h10 = 6*t - 4;
      h01 = -12*t + 6;
      h11 = 6*t-2;
   }
    
   return h00*position0 + h10*tangent0 + h01*position1 + h11*tangent1;
}
            
// Returns a state interpolated between the values directly before and after the given time.
template <class T>
inline T Spline<T>::evaluate( double time, int derivative )
{
   // TODO IMPLEMENT ME (TASK 1B)
   double t1, t2, normalizedTime;
   T tangent1, tangent2;
   T position1, position2;
   typename std::map<double, T>::iterator itr, itr1, itr2, itr3;

   if (!knots.empty()) {
      if (knots.size() == 1 && derivative != 0)
         return T();
      else if (knots.size() == 1 && derivative == 0)
         return knots.begin()->second;
      else {
         itr = knots.upper_bound(time);
      
         if ((itr == knots.begin() || itr == knots.end()) && derivative != 0)
            return T();
         else if (itr == knots.begin() && derivative == 0)
            return knots.begin()->second;
         else if (itr == knots.end() && derivative == 0)
            return knots.rbegin()->second;

         else {
            itr2 = itr;
            itr1 = std::prev(itr2);
            itr3 = std::next(itr2);
            position1 = itr1->second;
            position2 = itr2->second;
            tangent1 = position2 - position1;
            tangent2 = position2 - position1;
            t1 = itr1->first;
            t2 = itr2->first;
            normalizedTime = (time - t1)/(t2 - t1);

            if (itr3 != knots.end())
               tangent2 = (itr3->second - position1)/2;
            if (itr1 != knots.begin())
               tangent1 = (position2 - std::prev(itr1)->second)/2;
             
            T tmpRes = cubicSplineUnitInterval(position1, position2, tangent1, tangent2, normalizedTime, derivative);
             
            return tmpRes / pow(t2-t1, derivative);
         }
      }
   } else
      return T();
}

// Removes the knot closest to the given time,
//    within the given tolerance..
// returns true iff a knot was removed.
template <class T>
inline bool Spline<T>::removeKnot(double time, double tolerance )
{
   // Empty maps have no knots.
   if( knots.size() < 1 )
   {
      return false;
   }

   // Look up the first element > or = to time.
   typename std::map<double, T>::iterator t2_iter = knots.lower_bound(time);
   typename std::map<double, T>::iterator t1_iter;
   t1_iter = t2_iter;
   t1_iter--;

   if( t2_iter == knots.end() )
   {
      t2_iter = t1_iter;
   }

   // Handle tolerance bounds,
   // because we are working with floating point numbers.
   double t1 = (*t1_iter).first;
   double t2 = (*t2_iter).first;

   double d1 = fabs(t1 - time);
   double d2 = fabs(t2 - time);


   if(d1 < tolerance && d1 < d2)
   {
      knots.erase(t1_iter);
      return true;
   }

   if(d2 < tolerance && d2 < d1)
   {
      knots.erase(t2_iter);
      return t2;
   }

   return false;
}

// Sets the value of the spline at a given time (i.e., knot),
// creating a new knot at this time if necessary.
template <class T>
inline void Spline<T>::setValue( double time, T value )
{
   knots[ time ] = value;
}

template <class T>
inline T Spline<T>::operator()( double time )
{
   return evaluate( time );
}
