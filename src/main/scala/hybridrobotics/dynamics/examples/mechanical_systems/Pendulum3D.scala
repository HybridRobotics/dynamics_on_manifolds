package hybridrobotics.dynamics.examples.mechanical_systems

import java.io.{File, PrintWriter}

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
      val omega = Vec("Omega") // (d/dt)R = R*omega

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

      val eoms = computeEquationsOfMotion(L, infWork, configVars)
      println(s"Done: $eoms")

      // Generate txt file with latex equations
      val eom_latex = eoms._1
      val FILE_PATH = new java.io.File(".").getCanonicalPath
      val writer = new PrintWriter(new File(FILE_PATH + """\output\Pendulum3D.tex""" ))
      for (str <- eom_latex) {
          writer.write(str)
      }
      writer.close()
  }

}
