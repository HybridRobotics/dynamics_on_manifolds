// SphericalPendulum.scala
package dynamics_on_manifolds

import Differentiation._
import DynamicalModelComputation._
import Variation._

object SphericalPendulum {
    def main() {
      // define constant scalars
      val g = Cons("g")  // gravitational constant
      val m = Cons("m")  // point mass
      val l = Cons("l")  // length at which point mass hangs
   
      // define vectors
      val e3  = CVec( "e3")  // orientation of gravity
      val  q  = UVec(  "q")  // point mass acts on S^2
      val xi  =  Vec( "xi")  // vector orthoganal to q and dq
      val  u  =  Vec(  "u")  // virtual work done on system

      // set configuration variables (scalars,vectors,matrices)
      val configVars = Tuple3(List(),List(q),List())

       // define lagrangian
      val KE = Num(0.5) * m * l * l * (diffV(q) dot diffV(q))
      val PE = m * g * l * (q dot e3)
      val L = KE - PE
      
      // specify infinitesimal virtual work of system
      val infWork = Dot(deltaV(q),u)
      
      var eoms = computeEquationsOfMotion(L, infWork, configVars)
    }
}
