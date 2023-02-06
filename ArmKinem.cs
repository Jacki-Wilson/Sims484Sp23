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
        double magQSq;   // sum of squares of quaternions

        double kp;       // proportional gain
        double kd;       // derivative gain

        double psiDes;   // desired values of angles
        double thetaDes;
        double phiDes;
        double elbowAngleDes;
        double deg2Rad;  // converts degrees to radians
        double rad2Deg;  // converts radians to degrees

        double[,] R;     // Rotation matrix (dcm)
        double detR;     // Determinant of R, used for debugging purposes

        //--------------------------------------------------------------------
        // constructor
        //--------------------------------------------------------------------
        public ArmKinem() : base(8)
        {
            R = new double[3,3];
            detR = 33.3;
            magQSq = 22.2;

            kp = 200.0;
            kd = 20.0;

            psiDes = 0.0;
            thetaDes = 0.0;
            phiDes = 0.0;
            elbowAngleDes = 0.0;

            deg2Rad = Math.PI/180.0;
            rad2Deg = 1.0/deg2Rad;

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

            if(elbD < -120.0)
                elbD = -120.0;
            if(elbD > 0.0)
                elbD = 0.0;
            elbowAngleDes = deg2Rad*elbD;
        }

        //--------------------------------------------------------------------
        // PostProcess: calculate quaternions
        //--------------------------------------------------------------------
        public void PostProcess()
        {
            MapEulerYZXtoDCM();
            detR = DetRotationMatrix();
            MapRotationMat2Quat();
        }

        //--------------------------------------------------------------------
        // MapEulerYZXtoDCM
        //--------------------------------------------------------------------
        private void MapEulerYZXtoDCM()
        {
            double cPsi = Math.Cos(x[0]);
            double sPsi = Math.Sin(x[0]);
            double cTheta = Math.Cos(x[1]);
            double sTheta = Math.Sin(x[1]);
            double cPhi = Math.Cos(x[2]);
            double sPhi = Math.Sin(x[2]);

            R[0,0] = cPsi*cTheta;
            R[0,1] = sPhi*sPsi - sTheta*cPhi*cPsi;
            R[0,2] = sPhi*sTheta*cPsi + sPsi*cPhi;

            R[1,0] = sTheta;
            R[1,1] = cPhi*cTheta;
            R[1,2] = -sPhi*cTheta;

            R[2,0] = -sPsi*cTheta;
            R[2,1] = sPhi*cPsi+sPsi*sTheta*cPhi;
            R[2,2] = -sPhi*sPsi*sTheta + cPhi*cPsi;
        }

        //--------------------------------------------------------------------
        // DetRotationMatrix: Returns the determinant of the rotation matrix
        //--------------------------------------------------------------------
        private double DetRotationMatrix()
        {
            double det = R[0,0]*R[1,1]*R[2,2] + R[0,1]*R[1,2]*R[2,0] +
                R[0,2]*R[1,0]*R[2,1];
            
            det -= R[0,2]*R[1,1]*R[2,0] + R[0,1]*R[1,0]*R[2,2] + 
                R[0,0]*R[1,2]*R[2,1];

            return det;
        }

        //--------------------------------------------------------------------
        // MapRotationMat2Quat
        //--------------------------------------------------------------------
        private void MapRotationMat2Quat()
        {
            double tr = R[0,0] + R[1,1] + R[2,2];
            double s;

            if(tr > 0.0)
            {
                s = Math.Sqrt(tr + 1.0);
                q0 = s*0.5;
                s = 0.5/s;
                q1 = (-R[1,2] + R[2,1])*s;
                q2 = (-R[2,0] + R[0,2])*s;
                q3 = (-R[0,1] + R[1,0])*s;
                magQSq = q0*q0 + q1*q1 + q2*q2 + q3*q3;

                return;
            }

            if((R[0,0]>R[1,1]) && (R[0,0]>R[2,2]))
            {
                s = 2.0*Math.Sqrt(1.0+R[0,0]-R[1,1]-R[2,2]);
                q0 = (R[2,1] - R[1,2])/s;
                q1 = 0.25*s;
                q2 = (R[0,1] + R[1,0])/s;
                q3 = (R[0,2] + R[2,0])/s;
                magQSq = (q0*q0 + q1*q1 + q2*q2 + q3*q3);

                return;
            }

            if(R[1,1] > R[2,2])
            {
                s = 2.0*Math.Sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]);
                q0 = (R[0,2] - R[2,0])/s;
                q1 = (R[0,1] + R[1,0])/s;
                q2 = 0.25*s;
                q3 = (R[1,2] + R[2,1])/s;
                magQSq = (q0*q0 + q1*q1 + q2*q2 + q3*q3);

                return;
            }

            s = 2.0*Math.Sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]);
            q0 = (R[1,0] - R[0,1])/s;
            q1 = (R[0,2] + R[2,0])/s;
            q2 = (R[1,2] + R[2,1])/s;
            q3 = 0.25*s;
            magQSq = (q0*q0 + q1*q1 + q2*q2 + q3*q3);

            return;
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

        public double PsiDeg
        {
            get{ return x[0]*rad2Deg; }
        }

        public double ThetaDeg
        {
            get{ return x[1]*rad2Deg; }
        }

        public double PhiDeg
        {
            get{ return x[2]*rad2Deg; }
        }

        public double ElbowAngleDeg
        {
            get{ return x[3]*rad2Deg; }
        }

        public double DetRotMat
        {
            get{ return detR; }
        }

        public double Q0
        {
            get{ return q0;}
        }

        public double Q1
        {
            get{ return q1;}
        }

        public double Q2
        {
            get{ return q2;}
        }

        public double Q3
        {
            get{ return q3;}
        }

        public double QuatSquared
        {
            get{ return magQSq; }
        }
    }

    
}