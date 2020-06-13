package hybridrobotics.dynamics.calculus

import hybridrobotics.dynamics.data_types._
import Simplification._
import Expansion._

object LagrangeHamiltonianDynamics {

  def solveActionIntegral(Lagrangian: ScalarExpr, infWork: ScalarExpr,
                          configVariables: Tuple3[List[ScalarExpr], List[VectorExpr], List[MatrixExpr]]): Unit = {

    //    val startTime = System.nanoTime() // track computation time

    // sanity check
    val simplifyL1 = Lagrangian.basicSimplify()
    val simplifyL2 = basicSimplify(Lagrangian)

    // Convert Input Tree to Desired Format (terms expanded and combined numeric constants)
    var L = Lagrangian.basicSimplify()
    L = expansion(L)
//    L = simplify(L)

    println("Testing Done!")
  }

}
