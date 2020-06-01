package hybridrobotics.dynamics.mechanical_systems

import hybridrobotics.dynamics.operations.DynamicalModelComputation.computeEquationsOfMotion
import hybridrobotics.dynamics.operations._

object Pendulum3D
{

  def main()
  {

      // define constant scalars
      val g = Cons("g")  // gravitational constant
      val m = Cons("m")  // mass of pendulum

      // define vectors
      val e3    = CVec(  "e3") // direction of gravitational field
      val rho   = CVec( "rho") // vector, points to point mass with respect to R
      val eta   = Vec(  "eta") // generalized vector to describe variation on SO(3)
      val omega = Vec("omega") // (d/dt)R = R*omega

      // matrices
      val eta_skew  = SkewMat("eta") // skew matrix of generalized vector eta
      val R         = Mat(     "R") // special orthoganal matrix
      val J         = CSMat(   "J") // inertia matrix

      // set configuration variables
      val configVars = Tuple3(List(), List(), List(R))

      // define lagrangian
      val KE:Exp = Mul(Num(.5),Dot(omega,VMul(J,omega)))
      val PE:Exp = Mul(m,Mul(g,Dot(VMul(R,rho),e3)))

      val L = KE - PE

      // define infinitesimal work
      val M = Vec("M")
      val infWork = eta dot M

      var eoms = computeEquationsOfMotion(L, infWork, configVars)

  }

}
