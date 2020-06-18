package hybridrobotics.dynamics.examples.mechanical_systems

import java.io.{File, PrintWriter}

import hybridrobotics.dynamics.operations.Differentiation.diffV
import hybridrobotics.dynamics.operations.DynamicalModelComputation.computeEquationsOfMotion
import hybridrobotics.dynamics.operations._

object QuadrotorWithContinuum {

  def main() {
    // TODO fix the dynamics computation

    // define system parameters
    val g = Cons("g")
    val mq = Cons("m_q")
    val ma = Cons("m_a")
    val mi = Cons("m_i")
    val li = Cons("l_i")
    val lj = Cons("l_j")

    val e3 = CVec("e_3")
    val J = CSMat("J")

    // define state variables
    //
    val eta = Vec("eta")
    val Omega = Vec("Omega")

    val x = Vec("x")
    val xdot = diffV(x)

    val qi = UVec("q_i")
    val qidot = diffV(qi)
    val qj = UVec("q_j")
    val qjdot = diffV(qj)
    val xii = Vec("x_{ii}")
    val xij = Vec("x_{ij}")


    // define matrices
    val eta_skew = SkewMat("eta")
    val R = Mat("R")

    // set configuration variables
    val configVars = Tuple3(List(), List(qi), List(R))

    // define lagrangian
    val KE = Num(0.5) * (mq + mi) * Dot(xdot, xdot) + ma * li * Dot(xdot, qidot) + Num(0.5) * ma * li * lj * Dot(qidot, qjdot) + Num(0.5) * Dot(Omega, J ** Omega)
    val PE = (mq + mi) * g * Dot(x, e3) - ma * g * li * Dot(qi, e3)

    val L = KE - PE

    // define inputs
    val f = Var("f")
    val M = Vec("M")
    val infWork = (eta dot M)+ (deltaV(x) dot (R**e3*f))

    val eoms = computeEquationsOfMotion(L, infWork, configVars)
    println(s"Done: $eoms")

    // Generate txt file with latex equations
    val eom_latex = eoms._1
    val FILE_PATH = new java.io.File(".").getCanonicalPath
    val writer = new PrintWriter(new File(FILE_PATH + """\output\QuadrotorWithContinuum.tex""" ))
    for (str <- eom_latex) {
      writer.write(str)
    }
    writer.close()
  }
}
