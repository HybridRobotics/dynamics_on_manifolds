package hybridrobotics.dynamics.examples.mechanical_systems

import java.io.{File, PrintWriter}

import hybridrobotics.dynamics.operations.Differentiation.diffV
import hybridrobotics.dynamics.operations.DynamicalModelComputation.computeEquationsOfMotion
import hybridrobotics.dynamics.operations._
import hybridrobotics.dynamics.operations.PrintLine.print2LatexFile

object SphericalPendulum {

  def main() {

      // define constant scalars
      val g = Cons("g")  // gravitational constant
      val m = Cons("m")  // point mass
      val l = Cons("l")  // length at which point mass hangs

      // define vectors
      val e3  = CVec( "e3")  // orientation of gravity

      val  q  = UVec(  "q")  // point mass acts on S^2
      val xi  =  Vec( "xi")  // vector orthogonal to q and dq
      val u  =  Vec(  "u")  // virtual work done on system

      // set configuration variables (scalars,vectors,matrices)
      val configVars = Tuple3(List(),List(q),List())

       // define lagrangian
      val KE = Num(0.5) * m * l * l * (diffV(q) dot diffV(q))
      val PE = m * g * l * (q dot e3)
      val L = KE - PE

      // specify infinitesimal virtual work of system
      val infWork = Dot(deltaV(q),u)

      val eoms = computeEquationsOfMotion(L, infWork, configVars)
      print2LatexFile(eoms._1, "SphericalPendulum")
    }
}
