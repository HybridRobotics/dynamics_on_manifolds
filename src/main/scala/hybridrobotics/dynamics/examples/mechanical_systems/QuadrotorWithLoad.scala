package hybridrobotics.dynamics.examples.mechanical_systems

import java.io._

import hybridrobotics.dynamics.operations.Differentiation.diffV
import hybridrobotics.dynamics.operations.DynamicalModelComputation.computeEquationsOfMotion
import hybridrobotics.dynamics.operations._
import hybridrobotics.dynamics.operations.Simplification._
import hybridrobotics.dynamics.operations.PrintLine.print2LatexFile


object QuadrotorWithLoad {

  def main() {

    //
    // system parameters
    //
    // Constant scalars
    val g = Cons("g")
    val mQ = Cons("m_Q")
    val mL = Cons("m_L")
    val l = Cons("l")

    // define vectors
    val e3 = CVec("e_3")
    // define matrices
    val JQ = CSMat("J_Q")

    //
    // States
    //
    // Load position
    val xL = Vec("x_L")
    val xLdot = diffV(xL)
    // Load attitude
    val q = UVec("q")
    val qdot = diffV(q)
    // Quadrotor attitude
    val R = Mat("R")
    val eta = Vec("eta")
    val Omega = Vec("Omega")
    // Quadrotor position
    val xQ = xL-SMul(q, l)
    var xQdot = simplifyV(diffV(xQ))

    // set configuration variables
    val configVars = Tuple3(List(), List(xL, q), List(R))

    // define lagrangian
    val KE = Num(0.5) * mQ * Dot(xQdot , xQdot) + Num(0.5) * mL * Dot(xLdot, xLdot) + Num(0.5) * Dot(Omega, JQ ** Omega)
    val PE =  mL*g*Dot(xL, e3) +  mQ*g*Dot(xQ, e3)
    val L = KE - PE

    // define infinitesimal work
    val f = Var("f")
    val M = Vec("M")
    val infWork = (M dot eta) + ((deltaV(xL) - (deltaV(q) * l)) dot (R**e3*f)) // TODO fix the issue with deltaV(xQ)

    val eoms = computeEquationsOfMotion(L, infWork, configVars)
    println(s"Done: $eoms")

    // Generate txt file with latex equations
    print2LatexFile(eoms._1, "QuadrotorWithLoad")

  }
}
