package hybridrobotics.dynamics.examples.variation_linearization

import hybridrobotics.dynamics.calculus.MatrixManipulation.extractVariationCoefficients
import hybridrobotics.dynamics.calculus.PrintLine.{variationCoeffs2LatexEquation, print2LatexFile, printMLatex, printVLatex}
import hybridrobotics.dynamics.data_types._

object RigidPendulum {

  def main(): Unit = {

    //     Point-mass
    val filename: String = "VariationRigidPendulum"

    // Angular velocity SO3
    val m = ConstScalar("m")
    val g = ConstScalar("g")
    val e3 = ConstVector("e3")
    val R = SO3("R")
    val eta = R.getVariationVector
    val Om = R.getTangentVector
    val J = ConstMatrix("J")
    val u = Vector("u")
    val rho = ConstVector("\\rho")

    val equation = J ** Om.diff() + Cross(Om, J ** Om) + Cross(rho, MVMul(R.T, e3)) * m * g - u
    val var_equation = equation.delta()
    val variables = List(Om.diff().delta(), Om.delta(), eta, DeltaV(u))

    val startTime = System.nanoTime() // track computation time
    val coefficients = extractVariationCoefficients(var_equation, variables)
    val endTime = System.nanoTime()
    println("ComputationTime:" + (endTime - startTime) / 1000000)

    // Output
    val eqn_latex: String = variationCoeffs2LatexEquation(coefficients)
    println("%s", eqn_latex)

    var list_of_eqns: List[String] = List("$" + printVLatex(equation) + "=0$",
      "$" + printVLatex(var_equation) + "=0$",
      eqn_latex)

    print2LatexFile(list_of_eqns, filename)
    println("Testing Done!")
  }

}
