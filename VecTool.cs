//============================================================================
// VecTool.cs:  A set of simple tools for performing vector operations
//============================================================================
using System;

namespace Util484
{
    public class VecTool
    {
        double[] res;    // result vector

        //--------------------------------------------------------------------
        // Constructor for the class.
        //--------------------------------------------------------------------
        public VecTool()
        {
            res = new double[3];
        }

        //--------------------------------------------------------------------
        // AddTo:
        //--------------------------------------------------------------------
        public void AddTo(double[] v, double[] rr)
        {
            res = rr;

            res[0] += v[0];
            res[1] += v[1];
            res[2] += v[2];
        }

        public void AddTo(double[] v)
        {
            res[0] += v[0];
            res[1] += v[1];
            res[2] += v[2];
        }
    }
}