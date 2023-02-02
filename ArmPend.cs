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
using Util484;

namespace Sim
{
    public class ArmPend : Simulator
    {
        LinAlgEq linSys; // system of linear algebraic equations
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

        VecTool vt;      // vector tool changing basis, dot products, etc

        double[,] DCMS;  // DCM between shoulder frame and Newtonian frame
        double[,] DCME;  // DCM between elbow frame and Newtonian frame
        
        // partial velocities: first index -> generalized speed
        //                     second idx  -> body index
        //                     third index -> component in N frame 
        double[][][] pVel;
        double[][][] pAngVel;

        // body inertial terms: Coeffs of terms involving derivs of
        //       generalized speeds. DO NOT SWITCH SIGN.
        //       first index -> generalized speed for which deriv is taken
        //       second idx  -> body index
        //       third index -> component in N frame
        double[][][] inertiaBdyDeriv;

        // body inertial terms: Those NOT involving derivs of gen. speeds.
        //       These are full terms, not coefficients. DO NOT SWITCH SIGN.
        //       first index -> body index
        //       second idx  -> component in N frame
        double[][] inertiaBdyOther;

        // CG inertia terms (mass x accel): Coeffs of terms involving derivs
        //       of generalized speeds. DO NOT SWITCH SIGN.
        //       first index -> generalized speed for which deriv is taken
        //       second idx  -> body index
        //       third index -> commponent in N frame
        double[][][] inertiaCGDeriv;

        // CG inertia terms (mass x accel): Those parts NOT involving derivs
        //       of generalized speeds. DO NOT SWITCH SIGN.
        //       first index -> body index
        //       second idx  -> component in N frame
        double[][] inertiaCGOther;

        // Weight acting at body CG, moment on body
        //       first index -> body index
        //       second idx  -> component in N frame
        double[][] weight;
        double[][] moment;
        
        double tauE;  //torque at elbow
        double g;     // gravitational field strength

        // dummy vectors
        double[] dVec;
        double[] dVecB;

        // dummy variables
        double dumA;
        double dumB;
        double dumC;

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

            x[8] = 1.0;    // theta, elbow angle

            vt = new VecTool();

            DCMS = new double[3,3];
            DCME = new double[3,3];

            linSys = new LinAlgEq(4);

            // allocate arrays to hold patial velocities, inertia terms
            //     and generalized forces
            int i,j;
            pVel = new double[4][][];
            pAngVel = new double[4][][];
            inertiaBdyDeriv = new double[4][][];
            inertiaCGDeriv = new double[4][][];
            for(i=0;i<4;++i){
                pVel[i] = new double[2][];
                pAngVel[i] = new double[2][];
                inertiaBdyDeriv[i] = new double[2][];
                inertiaCGDeriv[i] = new double[2][];
                for(j=0;j<2;++j){
                    pVel[i][j] = new double[3];
                    pAngVel[i][j] = new double[3];
                    inertiaBdyDeriv[i][j] = new double[3];
                    inertiaCGDeriv[i][j] = new double[3];
                }
            }

            inertiaBdyOther = new double[2][];
            inertiaCGOther = new double[2][];
            weight = new double[2][];
            moment = new double[2][];
            for(j=0;j<2;++j){
                inertiaBdyOther[j] = new double[3];
                inertiaCGOther[j] = new double[3];
                weight[j] = new double[3];
                moment[j] = new double[3];
            }


            // partial angular velocity on upper arm due to elbow is zero
            pAngVel[3][0][0] = pAngVel[3][0][1] = pAngVel[3][0][2] = 0.0;
            
            // partial velocity of cm of upper arm due to omegaX is ZERO
            pVel[0][0][0] = pVel[0][0][1] = pVel[0][0][2] = 0.0;

            // partial vel of cm of upper arm due to elbow is ZERO
            pVel[3][0][0] = pVel[3][0][1] = pVel[3][0][2] = 0.0;

            // inertia upper arm body coefficient due to uTh is zero
            inertiaBdyDeriv[3][0][0] = inertiaBdyDeriv[3][0][1] = 
                inertiaBdyDeriv[3][0][2] = 0.0;

            // inertia upper arm CG coefficient due to omegaX is zero
            inertiaCGDeriv[0][0][0] = inertiaCGDeriv[0][0][1] = 
                inertiaCGDeriv[0][0][2] = 0.0;

            // inertia upper arm CG coefficient due to uTh is zero
            inertiaCGDeriv[3][0][0] = inertiaCGDeriv[3][0][1] = 
                inertiaCGDeriv[3][0][2] = 0.0;

            // weight
            g = 9.81;
            weight[0][0]=0.0; weight[0][1]=mAU*g; weight[0][2]=0.0; //AU
            weight[1][0]=0.0; weight[1][1]=mAL*g; weight[1][2]=0.0; //AL

            dVec = new double[3];
            dVecB = new double[3];

            tauE = 0.0;

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

            // generate DCMS (Shoulder)
            DCMS[0,0] = q0Sq + q1Sq - q2Sq - q3Sq;
            DCMS[0,1] = 2.0*(q1*q2 - q0*q3);
            DCMS[0,2] = 2.0*(q0*q2 + q1*q3);

            DCMS[1,0] = 2.0*(q0*q3 + q1*q2);
            DCMS[1,1] = q0Sq - q1Sq + q2Sq - q3Sq;
            DCMS[1,2] = 2.0*(q2*q3 - q0*q1);

            DCMS[2,0] = 2.0*(q1*q3 - q0*q2);
            DCMS[2,1] = 2.0*(q0*q1 + q2*q3);
            DCMS[2,2] = q0Sq - q1Sq - q2Sq + q3Sq;

            // generate DCME (Elbow)
            DCME[0,0] = DCMS[0,0]*cTh - DCMS[0,2]*sTh;
            DCME[0,1] = DCMS[0,1];
            DCME[0,2] = DCMS[0,0]*sTh + DCMS[0,2]*cTh;

            DCME[1,0] = DCMS[1,0]*cTh - DCMS[1,2]*sTh;
            DCME[1,1] = DCMS[1,1];
            DCME[1,2] = DCMS[1,0]*sTh + DCMS[1,2]*cTh;

            DCME[2,0] = DCMS[2,0]*cTh - DCMS[2,2]*sTh;
            DCME[2,1] = DCMS[2,1];
            DCME[2,2] = DCMS[2,0]*sTh + DCMS[2,2]*cTh;

            // calculate partial angular velocities, N frame
            pAngVel[0][0][0] = pAngVel[0][1][0] = DCMS[0,0];  // omX
            pAngVel[0][0][1] = pAngVel[0][1][1] = DCMS[1,0];
            pAngVel[0][0][2] = pAngVel[0][1][2] = DCMS[2,0];

            pAngVel[1][0][0] = pAngVel[1][1][0] = DCMS[0,1];  // omY
            pAngVel[1][0][1] = pAngVel[1][1][1] = DCMS[1,1];
            pAngVel[1][0][2] = pAngVel[1][1][2] = DCMS[2,1];

            pAngVel[2][0][0] = pAngVel[2][1][0] = DCMS[0,2];  // omZ
            pAngVel[2][0][1] = pAngVel[2][1][1] = DCMS[1,2];
            pAngVel[2][0][2] = pAngVel[2][1][2] = DCMS[2,2];

            pAngVel[3][1][0] = DCMS[0,1];  // uTheta
            pAngVel[3][1][1] = DCMS[1,1];
            pAngVel[3][1][2] = DCMS[2,1];

            // calculate partial velocities, N frame
            // uppper arm (omX) is zero, handled previously
            dVec[0] = dVec[1] = 0.0; dVec[2] = -dAU;  //upper arm (omY)
            vt.MapToFrame(DCMS,dVec,pVel[1][0]);
            dVec[0] = dVec[2] = 0.0; dVec[1] = dAU;   //upper arm (omZ)
            vt.MapToFrame(DCMS,dVec,pVel[2][0]);
            // upper arm (uTh) is zero, handled previously
            dVecB[0] = 0.0; dVecB[1] = dAL*sTh; dVecB[2] = 0.0; // AL (omX)
            vt.MapToFrame(DCME,dVecB,pVel[0][1]);
            dVec[0]=0.0; dVec[1]=0.0; dVec[2]=-LAU;  // lower arm (omY)
            dVecB[0]=0.0; dVecB[1]=0.0; dVecB[2]=-dAL;
            vt.MapToFrame(DCMS,dVec,pVel[1][1]);
            vt.MapToFrameAdd(DCME,dVecB);
            dVec[0]=0.0; dVec[1]=LAU; dVec[2]=0.0;   // lower arm (omZ)
            dVecB[0]=0.0; dVecB[1]=dAL*cTh; dVecB[2]=0.0;
            vt.MapToFrame(DCMS,dVec,pVel[2][1]);
            vt.MapToFrameAdd(DCME,dVecB);
            dVecB[0]=0.0; dVecB[1]=0.0; dVecB[2]=-dAL; // lower arm (uTh)
            vt.MapToFrame(DCME,dVecB,pVel[3][1]);

            // inertial body coefficients, N frame
            dVec[0]=IgAUx; dVec[1]=0.0; dVec[2]=0.0;   // upper arm (omX)
            vt.MapToFrame(DCMS,dVec,inertiaBdyDeriv[0][0]);
            dVec[0]=0.0; dVec[1]=IgAUy; dVec[2]=0.0;   // upper arm (omY)
            vt.MapToFrame(DCMS,dVec,inertiaBdyDeriv[1][0]);
            dVec[0]=0.0; dVec[1]=0.0; dVec[2]=IgAUz;   // upper arm (omZ)
            vt.MapToFrame(DCMS,dVec,inertiaBdyDeriv[2][0]);
            // upper arm (uTh) is zero, handled previously
            dVec[0]=IgALx*cTh; dVec[1]=0.0; dVec[2]=IgALz*sTh; // AL (omX)
            vt.MapToFrame(DCME,dVec,inertiaBdyDeriv[0][1]);
            dVec[0]=0.0; dVec[1]=IgALy; dVec[2]=0.0;  // lower arm (omY)
            vt.MapToFrame(DCME,dVec,inertiaBdyDeriv[1][1]);
            dVec[0]=-IgALx*sTh; dVec[1]=0.0; dVec[2]=IgALz*cTh; // AL (omZ)
            vt.MapToFrame(DCME,dVec,inertiaBdyDeriv[2][1]);
            dVec[0]=0.0; dVec[1]=IgALy; dVec[2]=0.0;  // lower arm (uTh)
            vt.MapToFrame(DCME,dVec,inertiaBdyDeriv[3][1]);

            // inertial body other terms, N frame
            dVec[0] = -(IgAUy-IgAUz)*omY*omZ;   //upper arm
            dVec[1] = -(IgAUz-IgAUx)*omZ*omX;
            dVec[2] = -(IgAUx-IgAUy)*omX*omY;
            vt.MapToFrame(DCMS,dVec,inertiaBdyOther[0]);

            dVecB[0] = omX*cTh - omZ*sTh; // ang vel of AL about e_x
            dVecB[1] = omY + uTh;         // ang vel of AL about e_y
            dVecB[2] = omX*sTh + omZ*cTh; // ang vel of AL about e_z
            dVec[0] = -IgALx*dVecB[2]*uTh - (IgALy-IgALz)*dVecB[1]*dVecB[2];
            dVec[1] = -(IgALz-IgALx)*dVecB[2]*dVecB[0];
            dVec[2] = IgALz*dVecB[0]*uTh - (IgALx-IgALy)*dVecB[0]*dVecB[1];
            vt.MapToFrame(DCME,dVec,inertiaBdyOther[1]);

            // inertia CG coefficients, N frame
            // upper Arm (omX) is zero, handled previously
            dVec[0]=0.0; dVec[1]=0.0; dVec[2]= -mAU*dAU;  // upper arm (omY)
            vt.MapToFrame(DCMS,dVec,inertiaCGDeriv[1][0]);
            dVec[0]=0.0; dVec[1]=mAU*dAU; dVec[2]=0.0;    // upper arm (omZ)
            vt.MapToFrame(DCMS,dVec,inertiaCGDeriv[2][0]);
            // upper Arm (uTh) is zero, handled previously
            dVecB[0]=0.0; dVecB[1]=mAL*dAL*sTh; dVecB[2]=0.0;  // AL (omX)
            vt.MapToFrame(DCME,dVecB,inertiaCGDeriv[0][1]);
            dVec[0]=0.0; dVec[1]=0.0; dVec[2]= -mAL*LAU;  // lower arm (omY)
            dVecB[0]=0.0; dVecB[1]=0.0; dVecB[2]= -mAL*dAL;
            vt.MapToFrame(DCMS,dVec,inertiaCGDeriv[1][1]);
            vt.MapToFrameAdd(DCME,dVecB);
            dVec[0]=0.0; dVec[1]=mAL*LAU; dVec[2]=0.0;   // lower arm (omZ)
            dVecB[0]=0.0; dVecB[1]=mAL*dAL*cTh; dVecB[2]=0.0;
            vt.MapToFrame(DCMS,dVec,inertiaCGDeriv[2][1]);
            vt.MapToFrameAdd(DCME,dVecB);
            dVecB[0]=0.0; dVecB[1]=0.0; dVecB[2]= -mAL*dAL; // lower arm (uTh)
            vt.MapToFrame(DCME,dVecB,inertiaCGDeriv[3][1]);

            // inertia CG other terms, N frame
            dVec[0] = -mAU*dAU*(omY*omY + omZ*omZ);    // upper arm
            dVec[1] = mAU*dAU*omX*omY;
            dVec[2] = mAU*dAU*omX*omZ;
            vt.MapToFrame(DCMS,dVec,inertiaCGOther[0]);
            dVec[0] = -mAL*LAU*(omY*omY + omZ*omZ);    // lower arm
            dVec[1] =  mAL*LAU*omX*omY;
            dVec[2] =  mAL*LAU*omX*omZ;
            dumA = omX*sTh + omZ*cTh;
            dumB = omY+uTh;
            dumC = omX*cTh - omZ*sTh;
            dVecB[0] = -mAL*dAL*(dumA*dumA + dumB*dumB);
            dVecB[1] = mAL*dAL*dumC*uTh;
            dVecB[2] = mAL*dAL*dumA*dumC;
            vt.MapToFrame(DCMS,dVec,inertiaCGOther[1]);
            vt.MapToFrameAdd(DCME,dVecB);

            // moment
            dVec[0]=0.0; dVec[1]=tauE; dVec[2]=0.0;   //AU
            vt.MapToFrame(DCMS,dVec,moment[0]);
            dVec[1]= -tauE;                           //AL
            vt.MapToFrame(DCMS,dVec,moment[1]);  

            // set up the equations of motion
            int i,j;
            linSys.SetABZero(); 
            for(i=0;i<4;++i)
            {
                for(j=0;j<4;++j)
                {
                    dumA =  vt.Dot(pAngVel[i][0], inertiaBdyDeriv[j][0]);
                    dumA += vt.Dot(pAngVel[i][1], inertiaBdyDeriv[j][1]);
                    dumA += vt.Dot(pVel[i][0], inertiaCGDeriv[j][0]);
                    dumA += vt.Dot(pVel[i][1], inertiaCGDeriv[j][1]);
                    linSys.A[i][j] = dumA;
                }

                dumB = -vt.Dot(pAngVel[i][0], inertiaBdyOther[0]);
                dumB -= vt.Dot(pAngVel[i][1], inertiaBdyOther[1]);
                dumB -= vt.Dot(pVel[i][0], inertiaCGOther[0]);
                dumB -= vt.Dot(pVel[i][1], inertiaCGOther[1]);

                dumB += vt.Dot(pAngVel[i][0], moment[0]);
                dumB += vt.Dot(pAngVel[i][1], moment[1]);
                dumB += vt.Dot(pVel[i][0], weight[0]);
                dumB += vt.Dot(pVel[i][1], weight[1]);

                linSys.b[i] = dumB;
            }
            linSys.SolveGauss();
            //TestFunc();

            ff[0] = linSys.sol[0];
            ff[1] = linSys.sol[1];
            ff[2] = linSys.sol[2];
            ff[3] = linSys.sol[3];

            // kinematic differential equations
            ff[4] = 0.5*(-q1*omX - q2*omY - q3*omZ);
            ff[5] = 0.5*( q0*omX - q3*omY + q2*omZ);
            ff[6] = 0.5*( q3*omX + q0*omY - q1*omZ);
            ff[7] = 0.5*(-q2*omX + q1*omY + q0*omZ);
            ff[8] = uTh;
        }


        //--------------------------------------------------------------------
        // TestFunc
        //--------------------------------------------------------------------
        public void TestFunc()
        {
            int i,j;

            Console.WriteLine("======================================");
            for(i=0;i<4;++i)
            {
                for(j=0;j<4;++j)
                {
                    Console.Write(linSys.A[i][j].ToString() + ", ");
                }
                Console.Write("      " + linSys.b[i].ToString() + "\n");
            }
            Console.WriteLine("--------------------------------------");
            for(i=0;i<4;++i)
                Console.WriteLine(linSys.sol[i].ToString());
            Console.WriteLine("--------inertia other-----------------");
            for(i=0;i<3;++i){
                Console.WriteLine(inertiaBdyOther[0][i].ToString() + "   " + 
                    inertiaBdyOther[1][i].ToString() + "   " +
                    inertiaCGOther[0][i].ToString() + "   " +
                    inertiaCGOther[1][i].ToString());

            }
            dumA = mAU*g*dAU;
            dumB = mAL*g*(LAU+dAL);
            Console.WriteLine("Moments   " + dumA.ToString() + "   " + dumB.ToString());

            dumA = IgAUz + mAU*dAU*dAU;
            dumB = IgALz + mAL*(LAU+dAL)*(LAU+dAL);
            Console.WriteLine("Rot Inertias   " + dumA.ToString() + "   " + dumB.ToString());
            Console.WriteLine("Masses   " + mAU.ToString() + "   " + mAL.ToString());
            Console.WriteLine("Lengths   " + LAU.ToString() + "  " + LAL.ToString());
            
            Console.WriteLine("======================================");
        }
    }
} 