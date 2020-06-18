package hybridrobotics.dynamics.examples.mechanical_systems

import hybridrobotics.dynamics.operations.DynamicalModelComputation.computeEquationsOfMotion
import hybridrobotics.dynamics.operations._

object DoublePendulum3D
{

    def main()
    {

        // define constant scalars
        val g =  Cons( "g")  // gravitational constant
        val m1 = Cons("m1")  // mass of pendulum 1
        val m2 = Cons("m2")  // mass of pendulum 2

        // define vectors
        val e3     = CVec(   "e3")
        val rhoc1  = CVec( "rhoc1")
        val rho1   = CVec(  "rho1")
        val rhoc2  = CVec( "rhoc1")
        val eta1   =  Vec(  "eta1")
        val eta2   =  Vec(  "eta2")
        val omega1 =  Vec("omega1")
        val omega2 =  Vec("omega2")

        // define matrices
        val eta_skew1   = SkewMat(  "eta1")
        val eta_skew2   = SkewMat(  "eta2")
        val omega_skew1 = SkewMat("omega1")
        val omega_skew2 = SkewMat("omega2")
        val R1          = Mat(       "R1")
        val R2          = Mat(       "R2")
        val J1          = CSMat(     "J1")
        val J2          = CSMat(     "J2")

        // set configuration variables
        val configVars = Tuple3(List(), List(), List(R1,R2))

        //Dot(u,VMul(MMul(MMul(r,SkewMat(s)),y),v))
        // Lagrangian is the difference between the kinetic energy (KE) and potential energy (PE).
        val KE:Exp = Num(.5)*Dot(omega1,VMul(J1,omega1)) + Num(.5)*Dot(omega2,VMul(J2,omega2)) + Num(0.5)*m1*(VMul(R1,VMul(omega_skew1,rhoc1)) dot VMul(R1,VMul(omega_skew1,rhoc1))) + Num(0.5)*m2*(VMul(R1,VMul(omega_skew1,rho1)) + VMul(R2,VMul(omega_skew2,rhoc2)) dot (VMul(R1,VMul(omega_skew1,rho1)) + VMul(R2,VMul(omega_skew2,rhoc2))))
        val PE:Exp = Mul(m1,Mul(g,Dot(VMul(R1,rhoc1),e3))) + Mul(m2,Mul(g,Dot(VMul(R1,rho1),e3))) + Mul(m2,Mul(g,Dot(VMul(R2,rhoc2),e3)))

        val L = KE - PE

        // Infinitesimal Work
        val infWork = Num(0)// == R^T dR dot M => R^T R*etaskew dot M => eta dot M

        var eoms = computeEquationsOfMotion(L, infWork, configVars)
    }

}
