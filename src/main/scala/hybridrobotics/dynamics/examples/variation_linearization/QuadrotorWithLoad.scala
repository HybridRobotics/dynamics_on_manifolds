package hybridrobotics.dynamics.examples.variation_linearization

import hybridrobotics.dynamics.data_types._
import hybridrobotics.dynamics.calculus.MatrixManipulation.extractVariationCoefficients
import hybridrobotics.dynamics.coder.Latex.{variationCoeffs2LatexEquation, print2LatexFile, printMLatex, printVLatex}

object QuadrotorWithLoad {

  def main(): Unit = {

    val filename: String = "VariationQuadrotorWithLoad"

    // System Parameters
    val mQ = ConstScalar("m_Q")
    val l = ConstScalar("l")
    val mL = ConstScalar("m_L")
    val g = ConstScalar("g")
    val e3 = ConstVector("e_3")
    val J = ConstMatrix("J")

    // states
    val x = Vector("x_L")
    val v = x.diff()
    val q = S2("q")
    val R = SO3("R")

    // inputs
    val f = VarScalar("f")
    val M = Vector("M")

//    //
//    // Variation linearization
//    //
//    // load_pos
//    val load_pos = SMul(v.diff() + e3 * g, (mQ + mL)) - SMul(q, Dot(q, R ** e3 * f) - Dot(q.diff(), q.diff()) * mQ * l)
//    val var_load_pos_raw = load_pos.delta()
//    val var_load_pos = var_load_pos_raw.basicSimplify()
//    val var_states = List(v.diff().delta(), x.delta(), v.delta(), q.getVariationVector, q.getTangentVector.delta(), R.getVariationVector, R.getTangentVector.delta())
//
//    val load_pos_coeffs = extractVariationCoefficients(var_load_pos, var_states)
//
//    // Output
//    var list_of_eqns: List[String] = List()
//    list_of_eqns = list_of_eqns :+ "Load Position Equation: \\begin{gather}\n" + printVLatex(load_pos) + "=0\n\\end{gather}"
//    list_of_eqns = list_of_eqns :+ "Variation load position equation: \\begin{gather}\n" + printVLatex(var_load_pos) + "=0\n\\end{gather}"
//    var load_var_eqn_latex: String = variationCoeffs2LatexEquation(load_pos_coeffs)
//    list_of_eqns = list_of_eqns :+ load_var_eqn_latex
//    print("%s\n", load_var_eqn_latex)
//
//    // load attitude
//    val om = q.getTangentVector
//    val load_att = om.diff() + Cross(q, R**e3*f)
//    val var_load_att_raw = load_att.delta()
//    val var_load_att = var_load_att_raw.basicSimplify()
//    val load_var_states = List(om.diff().delta(), x.delta(), v.delta(), q.getVariationVector, q.getTangentVector.delta(), R.getVariationVector, R.getTangentVector.delta())
//
//    val load_att_coeffs = extractVariationCoefficients(var_load_att, load_var_states)
//
//    // Output
//    list_of_eqns = list_of_eqns :+ "Load Attitude Equation: \\begin{gather}\n" + printVLatex(load_att) + "=0\n\\end{gather}"
//    list_of_eqns = list_of_eqns :+ "Variation load attitude equation: \\begin{gather}\n" + printVLatex(var_load_att) + "=0\n\\end{gather}"
//    var load_att_eqn_latex: String = variationCoeffs2LatexEquation(load_att_coeffs)
//    list_of_eqns = list_of_eqns :+ load_att_eqn_latex
//    print("%s\n", load_att_eqn_latex)
//
//    // quadrotor attitude
//    val Om = R.getTangentVector
//    val quad_att = J**Om.diff() + Cross(Om, J**Om) - M
//    val var_quad_att_raw = quad_att.delta()
//    val var_quad_att = var_quad_att_raw.basicSimplify()
//    val var_att_states = List(Om.diff().delta(), x.delta(), v.delta(), q.getVariationVector, q.getTangentVector.delta(), R.getVariationVector, R.getTangentVector.delta())
//
//    val quad_att_coeffs = extractVariationCoefficients(var_quad_att, var_att_states)
//
//    // Output
//    list_of_eqns = list_of_eqns :+ "Quad Attitude Equation: \\begin{gather}\n" + printVLatex(quad_att) + "=0\n\\end{gather}"
//    list_of_eqns = list_of_eqns :+ "Variation quad attitude equation: \\begin{gather}\n" + printVLatex(var_quad_att) + "=0\n\\end{gather}"
//    var quad_att_eqn_latex: String = variationCoeffs2LatexEquation(quad_att_coeffs)
//    list_of_eqns = list_of_eqns :+ quad_att_eqn_latex
//    print("%s\n", quad_att_eqn_latex)
//
//
//    print2LatexFile(list_of_eqns, filename)
//    println("Testing done")
  }

}
