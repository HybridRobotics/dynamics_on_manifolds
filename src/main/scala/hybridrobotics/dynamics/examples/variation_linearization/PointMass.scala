package hybridrobotics.dynamics.examples.variation_linearization

import hybridrobotics.dynamics.data_types._
import hybridrobotics.dynamics.calculus.MatrixManipulation.extractVariationCoefficients
import hybridrobotics.dynamics.coder.Latex.{variationCoeffs2LatexEquation, print2LatexFile, printMLatex, printVLatex}
import hybridrobotics.dynamics.coder._

object PointMass {

  def main(): Unit = {

    //     Point-mass
    val filename: String = "VariationPointMass"

    val x = Vector("x")
    val m = ConstScalar("m")
    val F = Vector("F")
    val acc = x.diff().diff()
    val equation = SMul(acc, m) - F
    val var_equation = equation.delta()

    val out: List[Any] = List(m, F)

    // Mergining all the variables
    val variables = (List(), List(acc.delta(), F.delta(), x.delta(), x.diff().delta()), List())

    val startTime = System.nanoTime() // track computation time
    val coefficients = extractVariationCoefficients(var_equation, variables)
    val endTime = System.nanoTime()
    println("ComputationTime:" + (endTime - startTime) / 1000000)


    val output_variables: List[Any] = List(acc.delta())
    val state_variables: List[Any] = List(x.delta(), x.diff().delta())
    val input_variables: List[Any] = List(F.delta())
    val dynamics = Matlab.generateLinearDynamics(List(coefficients), output_variables, state_variables, input_variables)


    // Output
    var eqn_latex: String = variationCoeffs2LatexEquation(coefficients)
    print(eqn_latex + "\n")


    // output function generation
    //    print2LatexFile(list_of_eqns2print, filename)
    Matlab.generateMatlabFunction(dynamics, filename)
    println("Testing Done!")
  }

}
