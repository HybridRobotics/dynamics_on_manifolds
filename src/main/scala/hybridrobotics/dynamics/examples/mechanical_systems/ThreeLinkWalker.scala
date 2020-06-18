package hybridrobotics.dynamics.examples.mechanical_systems

import hybridrobotics.dynamics.operations.Differentiation.diffV
import hybridrobotics.dynamics.operations.DynamicalModelComputation.computeEquationsOfMotion
import hybridrobotics.dynamics.operations._

object ThreeLinkWalker
{

  def main()
  {
      val n:Int = 3

      // system constants (scalars)
      val g = Cons("g")                // gravitational constant (earth)
      var m = Cons("m")
      var mt = Cons("mt")
      var mh = Cons("mh")
      var l = Cons("l")
      var LL = Cons("L")

      // vectors defined in system
      var e3   = Vec(  "e3")          // unit vector, points in direction of positive potential energy
      var q:List[VExp] = List()       // unit vector, points from pendulum attachment point to location of hanging mass
      var dq:List[VExp] = List()      // represents the variation of q
      var qdot:List[VExp] = List()    // vector orthoganal to q and dq
      var xi:List[VExp] = List()    // generalized vector xi

      for ( i <- 1 to n)
      {
          var qi = UVec("q"+i.toString)
          q  =  q :+ qi
          xi = xi :+ Vec("xi"+i.toString)
      }

      // set configuration variables
      val configVars = Tuple3(List(),q,List())

      // Lagrangian is the difference between the kinetic energy (KE) and potential energy (PE).
      var KE:Exp = Num(0.5)*(Num(1.25)*m+mt+mh)*l*l*Dot(diffV(q(0)),diffV(q(0))) + Num(0.5)*m*l*l*Dot(diffV(q(0)),diffV(q(1))) + Num(0.125)*m*l*l*Dot(diffV(q(1)),diffV(q(1))) + mh*l*LL*Dot(diffV(q(0)),diffV(q(2))) + Num(0.5)*mh*LL*LL*Dot(diffV(q(2)),diffV(q(2)))
      var PE:Exp = (Num(1.5)*m*g*l + mh*g*l + mt*g*l)*Dot(q(0),e3) + Num(0.5)*m*g*l*Dot(q(1),e3) + mh*g*LL*Dot(q(2),e3)
      var L = KE - PE

      // Infinitesimal Work of System
      val u2 = Vec("u2")
      val u3 = Vec("u3")
      val infWork = Dot(deltaV(q(1)),u2) + Dot(deltaV(q(2)),u3)

      computeEquationsOfMotion(L, infWork, configVars)



  }
}
