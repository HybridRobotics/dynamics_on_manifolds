package hybridrobotics.dynamics.examples.mechanical_systems

import hybridrobotics.dynamics.calculus.LagrangeHamiltonianDynamics._
import hybridrobotics.dynamics.data_types._
import hybridrobotics.dynamics.calculus.Simplification._
import hybridrobotics.dynamics.calculus.MatrixManipulation._
import hybridrobotics.dynamics.coder.Latex._


object ExampleSystem {

  def main(): Unit = {



    // Angular velocity SO3
    //    val R = SO3("R")
    //    val eta = R.getVariationVector
    //    val Om = R.getTangentVector
    //    val J = ConstMatrix("J")
    //    val M = Vector("M")
    //
    //    val ang_acc_eqn = J**Om.diff() + Cross(Om, J**Om) - M
    //    val var_ang_acc_eqn = ang_acc_eqn.delta()
    //    val variables = List(eta, Om.diff().delta(), Om.delta(), DeltaV(M))


    // Angular velocity S2
    val filename = "VariationS2Pend"
    val m = ConstScalar("m")
    val l = ConstScalar("l")
    val g = ConstScalar("g")
    val e3 = ConstVector("e_3")
    val q = S2("q")
    val om = q.getTangentVector
    val u = Vector("u")

    val equation = om.diff() * m * l * l - Cross(q, e3) * m * g * l - u
    val var_equation = equation.delta()
    val variables = List(om.diff().delta(),  q.getVariationVector, om.delta(), u.delta())

//    val coefficients = extractVariationCoefficients(var_equation, variables)
//
//
//    // Output
//    var eqn_latex: String = "$"
//    for ((k, v) <- coefficients) {
//      val ks : String = printVLatex(k)
//      val vs : String = printMLatex(v)
//      eqn_latex = eqn_latex + "\\Big[" + vs + "\\Big]" + ks + "+"
//    }
//    eqn_latex = eqn_latex + "=0$"
//
//    print("%s\n", eqn_latex)
//    print2LatexFile(List("$"+printVLatex(equation)+"=0$",
//                          "$"+printVLatex(var_equation)+"=0$",
//                          eqn_latex),
//                    filename)

    println("Testing Done!")

  }

  def derive_eom_example(): Unit = {

    // define constant scalars
    val g = ConstScalar("g") // gravitational constant
    val m = ConstScalar("m") // point mass
    val l = ConstScalar("l") // length at which point mass hangs

    // define vectors
    val e3 = ConstVector("e3") // orientation of gravity

    val q = S2("q") // point mass acts on S^2
    val xi = q.getVariationVector // vector orthogonal to q and dq
    val u = Vector("u") // virtual work done on system

    // set configuration variables (scalars,vectors,matrices)
    val configVars = Tuple3(List(), List(q), List())

    // define lagrangian
    val KE = (q.diff() dot q.diff()) * NumScalar(0.5) * m * l * l // Kinetic Energy
    val PE = m * g * l * (q dot e3) // Potential Energy
    val Lagrangian = KE - PE // Lagrangian

    // specify infinitesimal virtual work of system
    val infWork = Dot(q.delta(), u)


    val eoms = solveActionIntegral(Lagrangian, infWork, configVars)


    /** **************************************************/
    println("Testing Done!")
  }

  def test(): Unit = {
    //    val a = NumScalar(10.0)
    //    val b = NumScalar(11.0)
    //    val c = VarScalar("xy")
    //    val value = a*b //Mul(b,c)
    //
    //    val v2 = value.basicSimplify()
    //    val v1  = basicSimplify(value)


    //    //    val m = Matrix("sample")
    //    //    val mtrans = m.T
    //    //
    //    ////    def testFunc (e:Any) : Any = e match {
    //    ////      case e: MatrixExpr => Matrix(e.name+"MatrixExp")
    //    ////    }
    //    ////
    //    ////    val m2 = testFunc(m)
    //    //    val dm = m.diff()
    //    //    val mm2 = data_types.MMul(m,dm)
    //    //
    //    val a = Num(2.0)
    //    val b = Const("l")
    //    val c = Var("x")
    //
    //    val da = a.diff()
    //    val db = b.diff()
    //    val dc = c.diff()
    //
    //    val expr1 = a * (b + c)
    //    val dexpr1 = expr1.diff()
    //
    //    val v1 = Vector("q")
    //    val v2 = ConstVector("x")
    //
    //    val v1v2 = v1 dot v2
    //    val dotv1v2 = v1v2.diff()
    //    val deltav1v2 = v1v2.delta()
    //    //
    //    //    val mc = mtrans*c
    //    //    val mtranstrans = mtrans.T
    //    //
    //    //    val v = Vector("v")
    //    //    val vtrans = v.T
    //
    //    //    val q = S2("q")
    //    //    val deltaq = q.delta()
    //    //    val qdot  = q.diff()
    //    //
    //    //    val hatq = CrossMap(q)
    //    //    val veehatq = VeeMap(hatq)
    //    //
    //    val R = SO3("R")
    //    val dR = R.diff()
    //
    //    val d2R = dR.diff()
    //    //    val deltaR = R.delta()

  }

}
