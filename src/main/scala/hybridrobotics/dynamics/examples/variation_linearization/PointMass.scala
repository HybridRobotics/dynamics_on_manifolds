package hybridrobotics.dynamics.examples.variation_linearization

import hybridrobotics.dynamics.calculus.MatrixManipulation.extractVariationCoefficients
import hybridrobotics.dynamics.calculus.PrintLine.{print2LatexFile, printMLatex, printVLatex}
import hybridrobotics.dynamics.data_types._

object PointMass {

  def main(): Unit = {

    //     Point-mass
    val filename:String = "VariationPointMass"

    val x = Vector("x")
    val m = ConstScalar("m")
    val F = Vector("F")
    val acc = x.diff().diff()
    val equation = SMul(acc, m) - F
    val var_equation = equation.delta()
    val variables = List(acc.delta(), F.delta())

    val startTime = System.nanoTime() // track computation time
    val coefficients = extractVariationCoefficients(var_equation, variables)
    val endTime = System.nanoTime()
    println("ComputationTime:" + (endTime - startTime) / 1000000)

    // Output
    var eqn_latex: String = "$"
    for ((k, v) <- coefficients) {
      val ks: String = printVLatex(k)
      val vs: String = printMLatex(v)
      eqn_latex = eqn_latex + "\\Big[" + vs + "\\Big]" + ks + "+"
    }
    eqn_latex = eqn_latex + "=0$"
    print("%s\n", eqn_latex)

    val list_of_eqns2print: List[String] = List("$" + printVLatex(equation) + "=0$",
      "$" + printVLatex(var_equation) + "=0$",
      eqn_latex)

    print2LatexFile(list_of_eqns2print, filename)

    println("Testing Done!")
  }

}