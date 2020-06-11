package hybridrobotics.dynamics.mechanical_systems

import java.io.{File, PrintWriter}

import hybridrobotics.dynamics.operations.Differentiation.diffV
import hybridrobotics.dynamics.operations.DynamicalModelComputation.computeEquationsOfMotion
import hybridrobotics.dynamics.operations._

object Quadrotor {

  def main(): Unit = {

    // define system parameters
    val g = Cons("g")
    val m = Cons("m")
    val J = CSMat("J")

    // define constant vectors
    val e3     = CVec("e_3")

    // defining states
    val x = Vec("x")
    val v = diffV(x)
    val R = Mat("R")
    val Om = Vec("Omega")

    // variations
    val eta   = Vec("eta")
    val eta_skew   = SkewMat("eta")

    // inputs
    val f = Var("f")
    val M = Vec("M")

    // set configuration variables
    val configVars = Tuple3(List(), List(x), List(R))

    // define lagrangian
    val KE = Num(0.5)*m*Dot(v, v) + Num(0.5)*Dot(Om, J**Om)
    val PE = m*g*Dot(x, e3)

    // Lagrangian
    var L = KE - PE

    // define infinitesimal work)
    val infWork = (M dot eta) + (deltaV(x) dot (R**e3*f))

    val eoms = computeEquationsOfMotion(L, infWork, configVars)
    println(s"Done: $eoms")

    // Generate txt file with latex equations
    val eom_latex = eoms._1
    val FILE_PATH = new java.io.File(".").getCanonicalPath
    val writer = new PrintWriter(new File(FILE_PATH + """\output\Quadrotor.tex""" ))
    for (str <- eom_latex) {
      writer.write(str)
    }
    writer.close()
  }

}
