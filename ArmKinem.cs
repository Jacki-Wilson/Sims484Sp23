//============================================================================
// ArmKinem.cs
// Class for simulating the kinematics of a the left arm of our gymnast.
//============================================================================
using System;

namespace Sim
{
    public class ArmKinem : Simulator
    {
        double q0;       // Quaternion coordinates
        double q1;
        double q2;
        double q3;

        double kp;       // proportional gain
        double kd;       // derivative gain

        double psiDes;   // desired values of angles
        double thetaDes;
        double phiDes;
        double elbowAngleDes;

        double[,] R;     // Rotation matrix (dcm)

        //--------------------------------------------------------------------
        // constructor
        //--------------------------------------------------------------------
        public ArmKinem() : base(8)
        {
            R = new double[3,3];

            kp = 200.0;
            kd = 20.0;

            psiDes = 0.0;
            thetaDes = 0.0;
            phiDes = 0.0;
            elbowAngleDes = 0.0;

            // set up default initial conditions
            x[0] = 0.0;    // psi: rotation about N.y to form Frame A
            x[1] = 0.0;    // theta: rotation about A.z to form Frame B
            x[2] = 0.0;    // phi: rotation about B.x to form Frame C
            x[3] = 0.0;    // elbowAngle:
            x[4] = 0.0;    // psiDot
            x[5] = 0.0;    // thetaDot
            x[6] = 0.0;    // phiDot
            x[7] = 0.0;    // elbowAngleDot


        }

        //--------------------------------------------------------------------
        // RHSFunc: function which calculates the right side of the 
        //          differential equation
        //--------------------------------------------------------------------
        private void RHSFunc(double[] st, double t, double[] ff)
        {
            double psi = st[0];
            double theta = st[1];
            double phi = st[2];
            double elbowAngle = st[3];
            double psiDot = st[4];
            double thetaDot = st[5];
            double phiDot = st[6];
            double elbowAngleDot = st[7];

            ff[0] = psiDot;
            ff[1] = thetaDot;
            ff[2] = phiDot;
            ff[3] = elbowAngleDot;

            ff[4] = -kp*(psi - psiDes) - kd*psiDot;
            ff[5] = -kp*(theta - thetaDes) - kd*thetaDot;
            ff[6] = -kp*(phi - phiDes) - kd*phiDot;
            ff[7] = -kp*(elbowAngle - elbowAngleDes) - kd*elbowAngleDot;
        }
    }
}