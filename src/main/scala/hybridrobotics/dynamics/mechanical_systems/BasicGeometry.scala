package hybridrobotics.dynamics.mechanical_systems

import hybridrobotics.dynamics.operations._
import hybridrobotics.dynamics.operations.Variation.delta
//import hybridrobotics.dynamics.operations.SystemGeometry.{HatMap, VeeMap}
import hybridrobotics.dynamics.operations.Differentiation.diff
object BasicGeometry {

  def main () : Unit  = {

    val M = Mat("M")
    val R = SO3("R")
    val eta = R.getEta
    val deltaR = delta(R)
    val deltaM1 = delta(M)
    val delatM2 = deltaM(M)
    println(s""+R.s+"")

    val dotR = diff(R)

    println("Done!!!")

  }

}
