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
        double deg2Rad;  // converts degrees to radians

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

            deg2Rad = Math.PI/180.0;

            // set up default initial conditions
            x[0] = 0.0;    // psi: rotation about N.y to form Frame A
            x[1] = 0.0;    // theta: rotation about A.z to form Frame B
            x[2] = 0.0;    // phi: rotation about B.x to form Frame C
            x[3] = 0.0;    // elbowAngle:
            x[4] = 0.0;    // psiDot
            x[5] = 0.0;    // thetaDot
            x[6] = 0.0;    // phiDot
            x[7] = 0.0;    // elbowAngleDot

            SetRHSFunc(RHSFunc);
        }

        //--------------------------------------------------------------------
        // SetDesiredArmAngles:
        //--------------------------------------------------------------------
        public void SetDesiredArmAnglesDeg(double psiD, double thetaD, 
            double phiD, double elbD)
        {
            if(psiD < -120.0)
                psiD = -120.0;
            if(psiD > 80.0)
                psiD = 80.0;
            psiDes = deg2Rad*psiD;

            if(thetaD < -90.0)
                thetaD = -90.0;
            if(thetaD > 90.0)
                thetaD = 90.0;
            thetaDes = deg2Rad*thetaD;

            if(phiD < -90.0)
                phiD = -90.0;
            if(phiD > 90.0)
                phiD = 90.0;
            phiDes = deg2Rad*phiD;

            if(elbD < -90.0)
                elbD = -90.0;
            if(elbD > 90.0)
                elbD = 90.0;
            elbowAngleDes = deg2Rad*elbD;
        }

        //--------------------------------------------------------------------
        // MapEulerYZX2DCM
        //--------------------------------------------------------------------
        private void MapEulerYZX2DCM()
        {

        }

        //--------------------------------------------------------------------
        // RHSFunc: function which calculates the right side of the 
        //          differential equation.  THIS IS NOT THE WAY TO DO IT!
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

        //--------------------------------------------------------------------
        // Getters
        //--------------------------------------------------------------------
        public double Psi
        {
            get{ return x[0]; }
        }

        public double Theta
        {
            get{ return x[1]; }
        }

        public double Phi
        {
            get{ return x[2]; }
        }

        public double ElbowAngle
        {
            get{ return x[3]; }
        }
    }

    
}