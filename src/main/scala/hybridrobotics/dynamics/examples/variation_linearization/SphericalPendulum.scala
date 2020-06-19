package hybridrobotics.dynamics.examples.variation_linearization

import hybridrobotics.dynamics.calculus.MatrixManipulation.extractVariationCoefficients
import hybridrobotics.dynamics.calculus.PrintLine.{print2LatexFile, printMLatex, printVLatex}
import hybridrobotics.dynamics.data_types._

object SphericalPendulum {

  def main(): Unit = {

    val filename: String = "VariationSphericalPendulum"

    // System parameters
    val m = ConstScalar("m")
    val l = ConstScalar("l")
    val g = ConstScalar("g")

    val e3 = ConstVector("e_3")

    // States
    val q = S2("q")
    val om = q.getTangentVector
    val xi = q.getVariationVector

    val f = Vector("f")

    // Kinematic
    val var_kinematics = q.delta().diff() - q.diff().delta()
    var startTime = System.nanoTime() // track computation time
    val kinematics_coeff = extractVariationCoefficients(var_kinematics, List(xi.diff(), xi, om.delta()))
    var endTime = System.nanoTime()
    println("ComputationTime:" + (endTime - startTime) / 1000000)


    val dynamics = SMul(om.diff(), m*l)-Cross(q, f-e3*m*g)
    val var_dynamics = dynamics.delta()
    startTime = System.nanoTime() // track computation time
    val dynamics_coeff = extractVariationCoefficients(var_dynamics, List(om.delta().diff(), xi, om.delta(), f.delta()))
    endTime = System.nanoTime()
    println("ComputationTime:" + (endTime - startTime) / 1000000)

    // Output
    var eqn_latex: String = "$"
    for ((k, v) <- kinematics_coeff) {
      val ks: String = printVLatex(k)
      val vs: String = printMLatex(v)
      eqn_latex = eqn_latex + "\\Big[" + vs + "\\Big]" + ks + "+"
    }
    eqn_latex = eqn_latex + "=0$"
    print("%s\n", eqn_latex)
    var list_of_eqns: List[String] = List("$" + printVLatex(var_kinematics) + "=0$", eqn_latex)

    var dyn_eqn_latex: String = "$"
    for ((k, v) <- dynamics_coeff) {
      val ks: String = printVLatex(k)
      val vs: String = printMLatex(v)
      dyn_eqn_latex = dyn_eqn_latex + "\\Big[" + vs + "\\Big]" + ks + "+"
    }
    dyn_eqn_latex = dyn_eqn_latex + "=0$"
    print("%s\n", dyn_eqn_latex)
    list_of_eqns = list_of_eqns :+ "$" + printVLatex(dynamics) + "=0$"
    list_of_eqns = list_of_eqns :+ "$" + printVLatex(var_dynamics) + "=0$"
    list_of_eqns = list_of_eqns :+ dyn_eqn_latex


    print2LatexFile(list_of_eqns, filename)

    println("Testing done")

  }

}
