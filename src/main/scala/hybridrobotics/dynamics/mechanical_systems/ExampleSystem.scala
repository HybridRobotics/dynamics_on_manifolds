package hybridrobotics.dynamics.mechanical_systems


import hybridrobotics.dynamics.data_types
import hybridrobotics.dynamics.data_types._

object ExampleSystem {

  def main(): Unit = {
    //    val m = Matrix("sample")
    //    val mtrans = m.T
    //
    ////    def testFunc (e:Any) : Any = e match {
    ////      case e: MatrixExpr => Matrix(e.name+"MatrixExp")
    ////    }
    ////
    ////    val m2 = testFunc(m)
    //    val dm = m.diff()
    //    val mm2 = data_types.MMul(m,dm)
    //
    val a = Num(2.0)
    val b = Const("l")
    val c = Var("x")

    val da = a.diff()
    val db = b.diff()
    val dc = c.diff()

    val expr1 = a * (b + c)
    val dexpr1 = expr1.diff()
    //
    //    val mc = mtrans*c
    //    val mtranstrans = mtrans.T
    //
    //    val v = Vector("v")
    //    val vtrans = v.T

    //    val q = S2("q")
    //    val deltaq = q.delta()
    //    val qdot  = q.diff()
    //
    //    val hatq = CrossMap(q)
    //    val veehatq = VeeMap(hatq)
    //
    val R = SO3("R")
    val dR = R.diff()

    val d2R = dR.diff()
    //    val deltaR = R.delta()

    println("Testing Done!")
  }

}
