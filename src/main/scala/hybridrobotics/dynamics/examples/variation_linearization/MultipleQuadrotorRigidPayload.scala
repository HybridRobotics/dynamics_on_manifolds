package hybridrobotics.dynamics.examples.variation_linearization

import hybridrobotics.dynamics.data_types._
import hybridrobotics.dynamics.calculus.MatrixManipulation.extractVariationCoefficients
import hybridrobotics.dynamics.coder.Latex.{print2LatexFile, printVLatex, variationCoeffs2LatexEquation}
import hybridrobotics.dynamics.coder.Matlab

object MultipleQuadrotorRigidPayload {

  def main(): Unit = {

    val filename = "VariationMultipleQuadrotorRigidPayload"
    // System parameters
    val g = ConstScalar("g")
    val e3 = ConstVector("e_3")

    val mL = ConstScalar("m_L")
    val JL = ConstMatrix("J_L")

    val mi = ConstScalar("m_i")
    val Ji = ConstMatrix("J_i")

    val li = ConstScalar("l_i")
    val ri = ConstVector("r_i")

    // System States
    val xL = Vector("x_L")
    val vL = xL.diff()
    val RL = SO3("R_L")
    val etaL = RL.getVariationVector
    val OmL = RL.getTangentVector

    val qi = S2("q_i")
    val omi = qi.getTangentVector
    val xii = qi.getVariationVector

    val Ri = SO3("R_i")
    val etai = Ri.getVariationVector

    val fi = VarScalar("\\frac{f_i}{m_il_i}")

    val qib = RL.T ** qi

    // Dynamics 1 // TODO ignoring summation for now
    val A11 = IdentityMatrix() * mL + VVMul(qi, TransposeVector(qi)) * mi
    val A12 = VVMul(qi, TransposeVector(qib)) *** CrossMap(ri) * Mul(mi, NumScalar(-1.0))
    val lhs_dyn1 = A11 ** (vL.diff() + e3 * g) + A12 ** OmL.diff()

    val Gi = VVMul(qi, qi.T) * mi * li
    val ui = Ri ** e3 * fi
    val d1 = SMul(qi, Add(Dot(qib, MVMul(MMul(CrossMap(OmL), CrossMap(OmL)), ri)), Dot(omi, omi) * li) * Mul(mi, NumScalar(-1.0)))
    val rhs_dyn1 = Gi ** ui + d1

    val variables =  List(vL.diff().delta(), OmL.diff().delta(), xii, omi.delta(), etai, etaL, OmL.delta())

    var list_of_eqns: List[String] = List()
//
//    // d1
//    val var_d1_raw = d1.delta()
//    val var_d1 = var_d1_raw.basicSimplify()
//    val var_d1_coeffs = extractVariationCoefficients(var_d1, variables)
//
//    // Output
//    val var_d1_eqn_tex = variationCoeffs2LatexEquation(var_d1_coeffs)
////    print("\\begin{gather}\n" + printVLatex(d1) + "=0\n\\end{gather}")
////    print("\\begin{gather}\n" + printVLatex(var_d1) + "=0\n\\end{gather}")
//    print(var_d1_eqn_tex+"\n")
//
//    // Gi**ui
//    val Giui = Gi**ui
//    val var_Giui_raw = Giui.delta()
//    val var_Giui =  var_Giui_raw.basicSimplify()
//    val var_Giui_coeffs = extractVariationCoefficients(var_Giui, variables)
//
//    val var_Giui_eqn_tex = variationCoeffs2LatexEquation(var_Giui_coeffs)
//    print(var_Giui_eqn_tex+"\n")
//
//    // A11*(\dot vL +ge3)
//    val A11dVl = A11 ** (vL.diff() + e3 * g)
//    val var_A11dVl_raw = A11dVl.delta()
//    val var_A11dVl = var_A11dVl_raw.basicSimplify()
//    val var_A11dVl_coeffs= extractVariationCoefficients(var_A11dVl, variables)
//
//    val var_A11dVl_eqn_tex = variationCoeffs2LatexEquation(var_A11dVl_coeffs)
//    print(var_A11dVl_eqn_tex+"\n")
//
//    // A12 ** OmL.diff()
//    val A12dOml = A12 ** OmL.diff()
//    val var_A12doml_raw = A12dOml.delta()
//    val var_A12doml = var_A12doml_raw.basicSimplify()
//    val var_A12doml_coeffs = extractVariationCoefficients(var_A12doml, variables)
//
//    val var_A12_eqn_tex = variationCoeffs2LatexEquation(var_A12doml_coeffs)
//    print(var_A12_eqn_tex+"\n")
//
//
    var startTime = System.nanoTime() // track computation time
    val dyn_eqn1 = lhs_dyn1 - rhs_dyn1
    val var_dyn_eqn1 = dyn_eqn1.delta().basicSimplify()
    val coefficients = extractVariationCoefficients(var_dyn_eqn1, (List(), variables, List()))
    var endTime = System.nanoTime()
    println("ComputationTime:" + (endTime - startTime) / 1000000)

    val output_variables: List[Any] = List(vL.diff().delta(), OmL.diff().delta())
    val state_variables: List[Any] = List(OmL.diff().delta(), xii, omi.delta(), etai, etaL, OmL.delta())
    val input_variables: List[Any] = List()
    val dynamics = Matlab.generateLinearDynamics(List(coefficients), output_variables, state_variables, input_variables)


    // output function generation
    //    print2LatexFile(list_of_eqns2print, filename)
    Matlab.generateMatlabFunction(dynamics, filename)
    println("Testing Done!")

//
//    // Output
////    list_of_eqns = list_of_eqns :+ "Quad Attitude Equation: \\begin{gather}\n" + printVLatex(dyn_eqn1) + "=0\n\\end{gather}"
////    list_of_eqns = list_of_eqns :+ "Variation quad attitude equation: \\begin{gather}\n" + printVLatex(var_dyn_eqn1) + "=0\n\\end{gather}"
//    var eqn_latex: String = variationCoeffs2LatexEquation(dyn_eqn1_coeffs)
//
//
//    list_of_eqns = list_of_eqns :+ var_A11dVl_eqn_tex
//    list_of_eqns = list_of_eqns :+ var_A12_eqn_tex
//    list_of_eqns = list_of_eqns :+ var_Giui_eqn_tex
//    list_of_eqns = list_of_eqns :+ var_d1_eqn_tex
//    list_of_eqns = list_of_eqns :+ eqn_latex
//    print("%s\n", eqn_latex)
//
//    print2LatexFile(list_of_eqns, filename)
//    println("Done")

  }

}
