package hybridrobotics.dynamics.examples.variation_linearization

import hybridrobotics.dynamics.calculus.MatrixManipulation.extractVariationCoefficients
import hybridrobotics.dynamics.coder.Latex.{print2LatexFile, printMLatex, printVLatex, variationCoeffs2LatexEquation}
import hybridrobotics.dynamics.coder.Matlab
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
//
    val startTime = System.nanoTime() // track computation time
    val coefficients = extractVariationCoefficients(var_equation, (List(), variables, List()))
    val endTime = System.nanoTime()
    println("ComputationTime:" + (endTime - startTime) / 1000000)

    val output_variables: List[Any] = List(Om.diff().delta())
    val state_variables: List[Any] = List(eta, Om.delta())
    val input_variables: List[Any] = List(u.delta())
    val dynamics = Matlab.generateLinearDynamics(List(coefficients), output_variables, state_variables, input_variables)


    // output function generation
    //    print2LatexFile(list_of_eqns2print, filename)
    Matlab.generateMatlabFunction(dynamics, filename)
    println("Testing Done!")
  }

}
