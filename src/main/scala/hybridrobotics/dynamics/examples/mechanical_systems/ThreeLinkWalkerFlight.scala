package hybridrobotics.dynamics.examples.mechanical_systems

import hybridrobotics.dynamics.operations.Differentiation.diffV
import hybridrobotics.dynamics.operations.DynamicalModelComputation.computeEquationsOfMotion
import hybridrobotics.dynamics.operations._

object ThreeLinkWalkerFlight
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
    var x = Vec("x")


    for ( i <- 1 to n)
    {
        var qi = UVec("q"+i.toString)
        q  =  q :+ qi
        xi = xi :+ Vec("xi"+i.toString)
    }

    // set configuration variables
        val configVars = Tuple3(List(),q :+ x,List())

    // Lagrangian is the difference between the kinetic energy (KE) and potential energy (PE).
    var KE:Exp = Num(0.5)*(Num(1.25)*m+mt+mh)*Dot(x+(diffV(q(0))*l),x+(diffV(q(0))*l)) + Num(0.5)*m*Dot(x+(diffV(q(0))*l),x+(diffV(q(1))*l)) + Num(0.125)*m*Dot(x+(diffV(q(1))*l),x+(diffV(q(1))*l)) + mh*Dot(x+(diffV(q(0))*l),x+(diffV(q(2))*LL)) + Num(0.5)*mh*Dot(x+(diffV(q(0))*LL),x+(diffV(q(0))*LL))
    var PE:Exp = (Num(1.5)*m*g + mh*g + mt*g)*Dot(x+q(0)*l,e3) + Num(0.5)*m*g*Dot(x+q(1)*l,e3) + mh*g*Dot(x+q(2)*l,e3)
    var L = KE - PE

    // Infinitesimal Work of System
    val u2 = Vec("u2")
    val u3 = Vec("u3")
    val infWork = Dot(deltaV(q(1)),u2) + Dot(deltaV(q(2)),u3)

    computeEquationsOfMotion(L, infWork, configVars)
    }
}
