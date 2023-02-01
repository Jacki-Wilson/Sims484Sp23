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
        // MapToBasis
        //--------------------------------------------------------------------
        public void MapToBasis(double[,] dcm, double[] v, double[] rr)
        {
            res = rr;

            res[0] = dcm[0,0]*v[0] + dcm[0,1]*v[1] + dcm[0,2]*v[2];
            res[1] = dcm[1,0]*v[0] + dcm[1,1]*v[1] + dcm[1,2]*v[2];
            res[2] = dcm[2,0]*v[0] + dcm[2,1]*v[1] + dcm[2,2]*v[2];
        }

        public void MapToBasis(double[,] dcm, double[] v)
        {
            res[0] += dcm[0,0]*v[0] + dcm[0,1]*v[1] + dcm[0,2]*v[2];
            res[1] += dcm[1,0]*v[0] + dcm[1,1]*v[1] + dcm[1,2]*v[2];
            res[2] += dcm[2,0]*v[0] + dcm[2,1]*v[1] + dcm[2,2]*v[2];
        }

        //--------------------------------------------------------------------
        // SetBaseVector
        //--------------------------------------------------------------------
        public void SetBaseVector(double[] rr)
        {
            res = rr;
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

        public void AddTo(double sc,double[] v, double[] rr)
        {
            res = rr;

            res[0] += sc*v[0];
            res[1] += sc*v[1];
            res[2] += sc*v[2];
        }

        public void AddTo(double[] v)
        {
            res[0] += v[0];
            res[1] += v[1];
            res[2] += v[2];
        }

        public void AddTo(double sc,double[] v)
        {
            res[0] += sc*v[0];
            res[1] += sc*v[1];
            res[2] += sc*v[2];
        }

        //--------------------------------------------------------------------
        // SubtractFrom:
        //--------------------------------------------------------------------
        public void SubtractFrom(double[] v)
        {
            res[0] -= v[0];
            res[1] -= v[1];
            res[2] -= v[2];
        }

        public void SubtractFrom(double sc,double[] v)
        {
            res[0] -= sc*v[0];
            res[1] -= sc*v[1];
            res[2] -= sc*v[2];
        }
    }
}