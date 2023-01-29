//============================================================================
// ArmPend.cs
// Class for defining the dynamics of a double pendulum that models some of
// the joint dynamics of a human arm. The inner joint of the pendulum is a 
// spherical ball joint similar to the shoulder joint. The outer joint of the 
// double pendulum is a hinge joint, similar to an elbow.
//
// Data come from the following website: https://exrx.net/Kinesiology/Segments
//============================================================================
using System;

namespace Sim
{
    public class ArmPend : Simulator
    {
        double heightHuman = 1.731;    // human height (meters)
        double massHuman =  73.0;      // human mass (kg)

        double mAU;      // mass of upper arm
        double mAL;      // mass of lower arm
        double LAU;      // length of upper arm
        double LAL;      // length of lower arm
        double dAU;      // distance of cg of upper arm from shoulder
        double dAL;      // distance of cg of lower arm from elbow
        double IgAUx;    // moi of upper arm about CG, x component
        double IgAUy;    //                            y component
        double IgAUz;    //                            z component
        double IgALx;    // moi of lower arm about CG, x component
        double IgALy;    //                            y component
        double IgALz;    //                            z component

        double[,] DCMS;  // DCM between shoulder frame and Newtonian frame
        double[,] DCME;  // DCM between elbow frame and Newtonian frame

        //--------------------------------------------------------------------
        // constructor
        //--------------------------------------------------------------------
        public ArmPend() : base(9)
        {
            double rhoAU = 0.0271;   // mass fraction upper arm
            double rhoAF = 0.0162;   // mass fraction forearm
            double rhoH  = 0.0061;   // mass fraction of hand

            double xiAU = 0.172;     // length fraction upper arm
            double xiAF = 0.157;     // length fraction forearm
            double xiH  = 0.0575;    // length fraction of hand

            double etaAU = 0.5772;   // segment fraction of upper arm cg
            double etaAF = 0.4574;   // segment fraction of forearm cg
            double etaH = 0.79;      // segment fraction of hand

            double rFracAU = 0.15;   // radius/length upper arm
            double rFracAL = 0.1;    // radius/length lower arm

            mAU = rhoAU * massHuman;
            mAL = (rhoAF+rhoH) * massHuman;

            LAU = xiAU * heightHuman;
            LAL = (xiAF + xiH) *heightHuman;
            
            dAU = etaAU * LAU;
            dAL = (etaAF*xiAF*rhoAF +(xiAF + etaH*xiH)*rhoH)*
                heightHuman/(rhoAF+rhoH);

            double rAU = rFracAU*LAU;
            double rAL = rFracAL*LAL;

            IgAUx = 0.5*mAU*rAU*rAU;
            IgAUy = IgAUz = 0.25*mAU*rAU*rAU + mAU*LAU*LAU/12.0;

            IgALx = 0.5*mAL*rAL*rAL;
            IgALy = IgALz = 0.25*mAL*rAL*rAL + mAL*LAL*LAL/12.0;

            // set up default initial conditions
            x[0] = 0.0;    // omega_x, angular velocity of upper arm
            x[1] = 0.0;    // omega_y
            x[2] = 0.0;    // omega_z
            x[3] = 0.0;    // u_theta, time deriv of elbow angle

            x[4] = 1.0;    // q0, quaternion coords
            x[5] = 0.0;    // q1
            x[6] = 0.0;    // q2
            x[7] = 0.0;    // q3

            x[8] = 0.0;    // theta, elbow angle

            DCMS = new double[3,3];
            DCME = new double[3,3];

            SetRHSFunc(RHSFunc);

        }

        //--------------------------------------------------------------------
        // MapDCMtoQ: Take a direction cosine matrix and generate the 
        //      corresponding quaternion state
        //--------------------------------------------------------------------
        private void MapDCMtoQ(double[,] B)
        {

        }

        //--------------------------------------------------------------------
        // RHSFunc: function which calculates the right side of the 
        //          differential equation
        //--------------------------------------------------------------------
        private void RHSFunc(double[] st, double t, double[] ff)
        {
            double omX = st[0];
            double omY = st[1];
            double omZ = st[2];
            double uTh = st[3];
            double q0  = st[4];
            double q1  = st[5];
            double q2  = st[6];
            double q3  = st[7];
            double cTh = Math.Cos(st[8]);
            double sTh = Math.Sin(st[8]);

            double q0Sq = q0*q0;
            double q1Sq = q1*q1;
            double q2Sq = q2*q2;
            double q3Sq = q3*q3;

            // generate DCMS
            DCMS[0,0] = q0Sq + q1Sq - q2Sq - q3Sq;
            DCMS[0,1] = 2.0*(q1*q2 - q0*q3);
            DCMS[0,2] = 2.0*(q0*q2 + q1*q3);

            DCMS[1,0] = 2.0*(q0*q3 + q1*q2);
            DCMS[1,1] = q0Sq - q1Sq + q2Sq - q3Sq;
            DCMS[1,2] = 2.0*(q2*q3 - q0*q1);

            DCMS[2,0] = 2.0*(q1*q3 - q0*q2);
            DCMS[2,1] = 2.0*(q0*q1 + q2*q3);
            DCMS[2,2] = q0Sq - q1Sq - q2Sq + q3Sq;

            // generate DCME
            DCME[0,0] = DCMS[0,0]*cTh - DCMS[0,2]*sTh;
            DCME[0,1] = DCMS[0,1];
            DCME[0,2] = DCMS[0,0]*sTh + DCMS[0,2]*cTh;

            DCME[1,0] = DCMS[1,0]*cTh - DCMS[1,2]*sTh;
            DCME[1,1] = DCMS[1,1];
            DCME[1,2] = DCMS[1,0]*sTh + DCMS[1,2]*cTh;

            DCME[2,0] = DCMS[2,0]*cTh - DCMS[2,2]*sTh;
            DCME[2,1] = DCMS[2,1];
            DCME[2,2] = DCMS[2,0]*sTh + DCMS[2,2]*cTh;

        }
    }
} 