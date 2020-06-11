package hybridrobotics.dynamics.mechanical_systems

import java.io.{File, PrintWriter}

import hybridrobotics.dynamics.operations.Differentiation.diffV
import hybridrobotics.dynamics.operations.DynamicalModelComputation.computeEquationsOfMotion
import hybridrobotics.dynamics.operations._

object PointMass {

  def main(): Unit = {

    // define constant scalars
    val m = Cons("m")
    val g = Cons("g")

    // define vectors
    val e3  = CVec( "e_3")  // orientation of gravity
    val  x  =  Vec(  "x")  // point mass acts on R^3
    val  u  =  Vec(  "u")  // virtual work done on system

    val v = diffV(x)

    // set configuration variables (scalars,vectors,matrices)
    val configVars = Tuple3(List(),List(x),List())

    // define lagrangian
    val KE = Num(0.5) * m  * (v dot v)
    val PE = m * g *  (x dot e3)
    val L = KE - PE

    // specify infinitesimal virtual work of system
    val infWork = Dot(deltaV(x),u)

    val eoms = computeEquationsOfMotion(L, infWork, configVars)
    println(s"Done: $eoms")

    // Generate txt file with latex equations
    val eom_latex = eoms._1
    val FILE_PATH = new java.io.File(".").getCanonicalPath
    val writer = new PrintWriter(new File(FILE_PATH + """\output\PointMass.tex""" ))
    for (str <- eom_latex) {
      writer.write(str)
    }
    writer.close()

  }

}
