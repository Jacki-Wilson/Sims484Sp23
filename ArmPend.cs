//============================================================================
// ArmPend.cs
// Class for defining the dynamics of a double pendulum that models some of
// the joint dynamics of a human arm. The inner joint of the pendulum is a 
// spherical ball joint similar to the shoulder joint. The outer joint of the 
// double pendulum is a hinge joint, similar to an elbow.
//============================================================================
using System;

namespace Sim
{
    public class ArmPend
    {

        //--------------------------------------------------------------------
        // constructor
        //--------------------------------------------------------------------
        public ArmPend()
        {
            Console.WriteLine("ArmPend Constructor");
        }
    }
}