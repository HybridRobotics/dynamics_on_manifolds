package hybridrobotics.dynamics.mechanical_systems

import hybridrobotics.dynamics.operations.Differentiation.diffV
import hybridrobotics.dynamics.operations.DynamicalModelComputation.computeEquationsOfMotion
import hybridrobotics.dynamics.operations._

object QuadrotorWithLoad
{

  def main()
  {

    // define constant scalars
    val g = Cons("g")
    val m1 = Cons("mQ")
    val m2 = Cons("mL")
    val l2 = Cons("l")

    // define vectors
    var e3     = CVec(   "e3")
    var J1     = CSMat(   "J")
    var eta1   = Vec(   "eta1")
    var omega1 = Vec( "omega")
    var x1dot = Vec("xQdot")
    var x2dot = Vec("xLdot")
    var x2    = Vec("xL")
    var q2    = UVec("q")
    var xi2 = Vec("xi2")


    // define matrices
    var eta_skew   = SkewMat("eta")
    var R   = Mat("R")

    // set configuration variables
    val configVars = Tuple3(List(), List(x2,q2), List(R))

    // define lagrangian
    val KE = Num(0.5)*m1*Dot(x2dot-(diffV(q2)*l2),x2dot-(diffV(q2)*l2)) + Num(0.5)*m2*Dot(x2dot,x2dot) + Num(0.5)*Dot(omega1,J1**omega1)
    val PE = (m1+m2)*g*Dot(x2,e3) - m1*g*l2*Dot(q2,e3)

    var L = KE - PE

    // define infinitesimal work
    val f = Mat("f")
    val M1 = Vec("M1")
    val infWork = (eta1 dot M1) + ((deltaV(x2) - (deltaV(q2) * l2)) dot (f ** (R ** e3)))

    computeEquationsOfMotion(L, infWork, configVars)
  }
}
