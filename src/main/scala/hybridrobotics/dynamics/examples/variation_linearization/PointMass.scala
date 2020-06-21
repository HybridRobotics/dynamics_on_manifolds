package hybridrobotics.dynamics.examples.variation_linearization

import hybridrobotics.dynamics.data_types._
import hybridrobotics.dynamics.calculus.MatrixManipulation.extractVariationCoefficients
import hybridrobotics.dynamics.coder.Latex.{variationCoeffs2LatexEquation, print2LatexFile, printMLatex, printVLatex}
import hybridrobotics.dynamics.coder.Matlab._

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

    val output_variables: (List[ScalarExpr], List[VectorExpr], List[MatrixExpr]) = (List(), List(acc.delta()), List())
    val state_variables: (List[ScalarExpr], List[VectorExpr], List[MatrixExpr]) = (List(), List(), List())
    val input_variables: (List[ScalarExpr], List[VectorExpr], List[MatrixExpr]) = (List(), List(F.delta()), List())

    // Mergining all the variables
    val variables = (output_variables._1 ::: state_variables._1 ::: input_variables._1,
      output_variables._2 ::: state_variables._2 ::: input_variables._2,
      output_variables._3 ::: state_variables._3 ::: input_variables._3)

    val startTime = System.nanoTime() // track computation time
    val coefficients = extractVariationCoefficients(var_equation, variables)
    val endTime = System.nanoTime()
    println("ComputationTime:" + (endTime - startTime) / 1000000)



    //    // Output
    //    var eqn_latex: String = variationCoeffs2LatexEquation(coefficients)
    //    print(eqn_latex + "\n")
    //    val list_of_eqns2print: List[String] = List("$" + printVLatex(equation) + "=0$",
    //      "$" + printVLatex(var_equation) + "=0$",
    //      eqn_latex)
    //
    //
    //    var matlabcode : String = ""
    //    for ((k, v) <- coefficients) {
    //      val ks: String = vectorExpr2Matlab(k)
    //      val vs: String = matrixExpr2Matlab(v)
    //      matlabcode = matlabcode + "["+vs+"]"+ks
    //    }
    //    print(matlabcode+"\n")


    // output function generation
    //    print2LatexFile(list_of_eqns2print, filename)
    generateMatlabFunction(filename)
    println("Testing Done!")
  }

}
