package hybridrobotics.dynamics.operations

import hybridrobotics.dynamics.operations._
import Expansion._
import ApplyRules._
import Cancellation._
import Differentiation._
import IntegrationByParts._
import PrintLine._
import Simplification._
import Variation._

object ErrorDynamics {

  def computeErrorDynamics(eom: Tuple3[List[Exp], List[VExp], List[MExp]], Rules: Tuple3[Map[Exp, Exp], Map[VExp, VExp], Map[MExp, MExp]]) {
    val startTime = System.nanoTime() // Track computation time
    println("Error Dynamics:")

    for (s <- eom._1) {
      // Take Variation with Respect to Desired Trajectory
      var rhs = variation_d(s)


      // Substitute in for variations
      rhs = applyConstraints(rhs, Rules._1, Rules._2, Rules._3)
      rhs = expansion(rhs)
      rhs = simplify(rhs)

      println(printTree(rhs))
    }

    for (v <- eom._2) {
      // Take Variation with Respect to Desired Trajectory
      var rhs = variationV_d(v)

      // Substitute for variations
      rhs = applyConstraintsV(rhs, Rules._2, Rules._3)
      rhs = expandV(rhs)
      rhs = simplifyV(rhs)

      println(printVTree(rhs))
    }

    for (v <- eom._3) {
      // Take Variation with Respect to Desired Trajectory
      var rhs = variationM_d(v)


      // Substitute for variations
      rhs = applyConstraintsM(rhs, Rules._3)
      rhs = expandM(rhs)
      println(printMTree(rhs))
    }
    val endTime = System.nanoTime()
    println("ComputationTime:" + (endTime - startTime) / 1000000)
  }
}
