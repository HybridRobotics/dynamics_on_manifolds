package hybridrobotics.dynamics.calculus

import hybridrobotics.dynamics.data_types._
import Simplification._
import Expansion._
import SystemGeometry._

object LagrangeHamiltonianDynamics {

  def solveActionIntegral(Lagrangian: ScalarExpr, infWork: ScalarExpr,
                          configVariables: Tuple3[List[ScalarExpr], List[VectorExpr], List[MatrixExpr]]): Unit = {

    //    val startTime = System.nanoTime() // track computation time

    // sanity check
    val simplifyL1 = Lagrangian.basicSimplify()
    val simplifyL2 = basicSimplify(Lagrangian)

    // Convert Input Tree to Desired Format (terms expanded and combined numeric constants)
    var L = Lagrangian.basicSimplify() // simplify
    L = expansion(L)
    L = simplifyScalarExpr(L)

    // Step 0: Generate Collection Terms and Constraints of the Configuration Manifold
    // Check for Geometry Dependent Equations of Motion
    var colTerms = generateCollectionTerms(configVariables)

    println("Testing Done!")
  }

}
