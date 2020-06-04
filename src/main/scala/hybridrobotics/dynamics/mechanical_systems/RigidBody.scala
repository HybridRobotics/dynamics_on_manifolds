package hybridrobotics.dynamics.mechanical_systems

import hybridrobotics.dynamics.operations.Differentiation.diffV
import hybridrobotics.dynamics.operations.DynamicalModelComputation.computeEquationsOfMotion
import hybridrobotics.dynamics.operations._

object RigidBody {

  def main(): Unit = {

    // define system parameters
    val g = Cons("g")
    val m = Cons("m")
    val J = CSMat("J")

    // define constant vectors
    val e3     = CVec("e3")

    // defining states
    val x = Vec("x")
    val v = diffV(x)
    val R = Mat("R")
    val Om = Vec("Omega")

    // variations
    val eta   = Vec("eta")
    val eta_skew   = SkewMat("eta")

    // inputs
    val F = Vec("F")
    val M = Vec("M")

    // set configuration variables
    val configVars = Tuple3(List(), List(x), List(R))

    // define lagrangian
    val KE = Num(0.5)*m*Dot(v, v) + Num(0.5)*Dot(Om, J**Om)
    val PE = m*g*Dot(x, e3)

    // Lagrangian
    var L = KE - PE

    // define infinitesimal work)
    val infWork = (M dot eta) + (deltaV(x) dot F)

    computeEquationsOfMotion(L, infWork, configVars)
  }

}
