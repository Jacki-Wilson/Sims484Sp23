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
            dAL = (etaAF*xiAF*rhoAF +(xiAF + etaH*xiH)*rhoH)*heightHuman/(rhoAF+rhoH);

            double rAU = rFracAU*LAU;
            double rAL = rFracAL*LAL;

            IgAUx = 0.5*mAU*rAU*rAU;
            IgAUy = IgAUz = 0.25*mAU*rAU*rAU + mAU*LAU*LAU/12.0;

            IgALx = 0.5*mAL*rAL*rAL;
            IgALy = IgALz = 0.25*mAL*rAL*rAL + mAL*LAL*LAL/12.0;

            //Console.WriteLine("Masses: " + mAU.ToString() + ", " + mAL.ToString());
            //Console.WriteLine("Lengths: " + LAU.ToString() + ", " + LAL.ToString());
            //Console.WriteLine("dCG: " + dAU.ToString() + ", " + dAL.ToString());
        }
    }
} 